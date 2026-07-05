#!/usr/bin/env python3
"""
Read diagnostic data from a Bluetti AC180 over Bluetooth Low Energy.

Install:
  python3 -m pip install bleak paho-mqtt

Examples:
  python3 bluetti_ac180_reader.py scan
  python3 bluetti_ac180_reader.py dump --name AC180 --seconds 20 --jsonl ac180_raw.jsonl
  python3 bluetti_ac180_reader.py poll --address AA:BB:CC:DD:EE:FF --start 0 --count 120
  python3 bluetti_ac180_reader.py mqtt --broker 192.168.1.10 --map ac180_map.json

Notes:
  Bluetti stations commonly expose a BLE service ff00 with notify/write
  characteristics ff01/ff02 and Modbus-like register reads. Firmware revisions
  can differ, so this script saves raw frames as well as decoded register words.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = None
    BleakScanner = None

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


BLUETTI_SERVICE_UUID = "0000ff00-0000-1000-8000-00805f9b34fb"
BLUETTI_NOTIFY_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"
BLUETTI_WRITE_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"


def now_ms() -> int:
    return int(time.time() * 1000)


def hex_bytes(data: bytes) -> str:
    return data.hex(" ")


def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(data: bytes) -> bytes:
    crc = modbus_crc(data)
    return data + struct.pack("<H", crc)


def check_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    return modbus_crc(frame[:-2]) == struct.unpack("<H", frame[-2:])[0]


def read_registers(start: int, count: int, unit: int = 1, function_code: int = 3) -> bytes:
    if not 0 <= start <= 0xFFFF:
        raise ValueError("start must be in 0..65535")
    if not 1 <= count <= 125:
        raise ValueError("count must be in 1..125")
    if function_code not in {3, 4}:
        raise ValueError("function code must be 3 or 4")
    payload = struct.pack(">BBHH", unit, function_code, start, count)
    return append_crc(payload)


def read_holding_registers(start: int, count: int, unit: int = 1) -> bytes:
    return read_registers(start, count, unit=unit, function_code=3)


def decode_modbus_register_response(frame: bytes) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "raw_hex": hex_bytes(frame),
        "crc_ok": check_crc(frame),
    }
    if len(frame) < 5:
        result["error"] = "frame too short"
        return result

    unit = frame[0]
    function = frame[1]
    result["unit"] = unit
    result["function"] = function

    if function & 0x80:
        result["exception"] = frame[2] if len(frame) > 2 else None
        return result

    if function not in {0x03, 0x04}:
        result["error"] = "not a read-registers response"
        return result

    byte_count = frame[2]
    data = frame[3 : 3 + byte_count]
    if len(data) != byte_count:
        result["error"] = "truncated register payload"
        return result
    if byte_count % 2:
        result["error"] = "odd register byte count"
        return result

    words = [int.from_bytes(data[i : i + 2], "big") for i in range(0, len(data), 2)]
    signed_words = [w - 0x10000 if w & 0x8000 else w for w in words]
    result["register_count"] = len(words)
    result["registers_u16"] = words
    result["registers_i16"] = signed_words
    result["heuristics"] = heuristic_decode(words)
    return result


def load_map(path: Optional[str]) -> Dict[str, Any]:
    if not path:
        return {"fields": []}
    with Path(path).open("r", encoding="utf-8") as file:
        mapping = json.load(file)
    if not isinstance(mapping.get("fields", []), list):
        raise ValueError("map file must contain a 'fields' list")
    return mapping


def register_value(words: List[int], index: int, value_type: str, word_order: str) -> Any:
    if value_type == "u16":
        return words[index]
    if value_type == "i16":
        value = words[index]
        return value - 0x10000 if value & 0x8000 else value
    if value_type in {"u32", "i32"}:
        pair = words[index : index + 2]
        if len(pair) != 2:
            raise IndexError("not enough registers for 32-bit value")
        if word_order == "little":
            pair = [pair[1], pair[0]]
        value = (pair[0] << 16) | pair[1]
        if value_type == "i32" and value & 0x80000000:
            return value - 0x100000000
        return value
    raise ValueError(f"unsupported value type: {value_type}")


def words_to_bytes(words: List[int]) -> bytes:
    data = bytearray()
    for word in words:
        data.extend(int(word).to_bytes(2, "big"))
    return bytes(data)


def swapped_pairs(data: bytes) -> bytes:
    swapped = bytearray(data)
    for index in range(0, len(swapped) - 1, 2):
        swapped[index], swapped[index + 1] = swapped[index + 1], swapped[index]
    return bytes(swapped)


def decode_string_words(words: List[int], swapped: bool = False) -> str:
    data = words_to_bytes(words)
    if swapped:
        data = swapped_pairs(data)
    return data.rstrip(b"\0").decode("ascii", errors="replace")


def parse_registers(start: int, words: List[int], mapping: Dict[str, Any]) -> Dict[str, Any]:
    return parse_register_map(registers_by_address(start, words), mapping)


def parse_register_map(registers: Dict[str, int], mapping: Dict[str, Any]) -> Dict[str, Any]:
    parsed: Dict[str, Any] = {}
    for field in mapping.get("fields", []):
        if "formula" in field:
            continue
        name = field["name"]
        address = int(field["address"])
        value_type = field.get("type", "u16")
        word_order = field.get("word_order", "big")
        size = int(field.get("size", 1))
        scale = field.get("scale", 1)
        offset = field.get("offset", 0)

        if str(address) not in registers:
            continue

        try:
            value = register_value_from_map(registers, address, value_type, word_order, size)
        except IndexError:
            continue

        if value_type in {"string", "swap_string"}:
            pass
        elif "bit" in field:
            value = bool((int(value) >> int(field["bit"])) & 1)
        elif "equals" in field:
            value = value == field["equals"]
        elif "enum" in field:
            value = field["enum"].get(str(value), value)
        else:
            value = (value * scale) + offset
            if isinstance(value, float):
                value = round(value, int(field.get("round", 3)))

        parsed[name] = {
            "value": value,
            "address": address,
            "type": value_type,
        }
        if "unit" in field:
            parsed[name]["unit"] = field["unit"]
    parsed.update(parse_computed_fields(parsed, mapping))
    return parsed


def parse_computed_fields(parsed: Dict[str, Any], mapping: Dict[str, Any]) -> Dict[str, Any]:
    computed: Dict[str, Any] = {}
    for field in mapping.get("fields", []):
        formula = field.get("formula")
        if not formula:
            continue

        name = field["name"]
        sources = field.get("sources", [])
        if any(source not in parsed for source in sources):
            continue

        source_values = [parsed[source]["value"] for source in sources]
        if formula == "multiply":
            value = 1
            for item in source_values:
                value *= item
        elif formula == "sum":
            weights = field.get("weights") or [1] * len(source_values)
            if len(weights) != len(source_values):
                continue
            value = sum(item * weight for item, weight in zip(source_values, weights))
        else:
            continue

        value = (value * field.get("scale", 1)) + field.get("offset", 0)
        if isinstance(value, float):
            value = round(value, int(field.get("round", 3)))

        computed[name] = {
            "value": value,
            "type": "computed",
            "sources": sources,
        }
        if "unit" in field:
            computed[name]["unit"] = field["unit"]
        if "note" in field:
            computed[name]["note"] = field["note"]
    return computed


def registers_by_address(start: int, words: List[int]) -> Dict[str, int]:
    return {str(start + offset): value for offset, value in enumerate(words)}


def register_value_from_map(registers: Dict[str, int], address: int, value_type: str, word_order: str, size: int = 1) -> Any:
    if value_type in {"u16", "i16"}:
        return register_value([registers[str(address)]], 0, value_type, word_order)
    if value_type in {"u32", "i32"}:
        if str(address + 1) not in registers:
            raise IndexError("not enough registers for 32-bit value")
        return register_value([registers[str(address)], registers[str(address + 1)]], 0, value_type, word_order)
    if value_type in {"string", "swap_string"}:
        words = []
        for offset in range(size):
            key = str(address + offset)
            if key not in registers:
                raise IndexError("not enough registers for string value")
            words.append(registers[key])
        return decode_string_words(words, swapped=value_type == "swap_string")
    raise ValueError(f"unsupported value type: {value_type}")


def parse_ranges(args: argparse.Namespace) -> List[tuple]:
    if not getattr(args, "ranges", None):
        return [(args.start, args.count)]

    ranges = []
    for item in args.ranges:
        try:
            start_text, count_text = item.split(":", 1)
            ranges.append((int(start_text), int(count_text)))
        except ValueError:
            raise ValueError(f"invalid range '{item}', expected START:COUNT")
    return ranges


def heuristic_decode(words: List[int]) -> Dict[str, Any]:
    """Conservative guesses for common power-station telemetry fields.

    The exact AC180 register map may vary. These values are intentionally
    labelled as guesses until calibrated against the Bluetti app display.
    """
    guessed: Dict[str, Any] = {}
    plausible_percent = [w for w in words if 0 <= w <= 100]
    plausible_watts = [w - 0x10000 if w & 0x8000 else w for w in words]
    plausible_watts = [w for w in plausible_watts if -3000 <= w <= 3000]
    plausible_decivolts = [w / 10 for w in words if 100 <= w <= 3000]

    if plausible_percent:
        guessed["possible_percent_values"] = plausible_percent[:12]
    if plausible_watts:
        guessed["possible_watt_values"] = plausible_watts[:20]
    if plausible_decivolts:
        guessed["possible_voltage_values_x0.1"] = plausible_decivolts[:12]
    return guessed


@dataclass
class Target:
    address: str
    name: Optional[str]


async def find_target(address: Optional[str], name: Optional[str], timeout: float) -> Target:
    if BleakScanner is None:
        raise RuntimeError("bleak is not installed. Run: python3 -m pip install bleak")
    if address:
        return Target(address=address, name=None)

    devices = await BleakScanner.discover(timeout=timeout, return_adv=True)
    candidates: List[Target] = []
    needle = (name or "bluetti").lower()

    for device, adv in devices.values():
        names = [
            device.name or "",
            adv.local_name or "",
            *(adv.service_data.keys() if adv.service_data else []),
        ]
        joined = " ".join(names).lower()
        if needle in joined or "bluetti" in joined or "ac180" in joined:
            candidates.append(Target(address=device.address, name=device.name or adv.local_name))

    if not candidates:
        raise RuntimeError(
            "No Bluetti-like BLE device found. Try: scan, then pass --address."
        )
    return candidates[0]


async def scan(args: argparse.Namespace) -> None:
    if BleakScanner is None:
        raise RuntimeError("bleak is not installed. Run: python3 -m pip install bleak")
    devices = await BleakScanner.discover(timeout=args.timeout, return_adv=True)
    rows = []
    for device, adv in devices.values():
        rows.append(
            {
                "address": device.address,
                "name": device.name or adv.local_name or "",
                "rssi": adv.rssi,
                "service_uuids": list(adv.service_uuids or []),
            }
        )
    rows.sort(key=lambda row: (("bluetti" not in row["name"].lower()), -row["rssi"]))
    print(json.dumps(rows, indent=2))


async def dump(args: argparse.Namespace) -> None:
    if BleakClient is None:
        raise RuntimeError("bleak is not installed. Run: python3 -m pip install bleak")
    target = await find_target(args.address, args.name, args.timeout)
    print(f"Connecting to {target.name or target.address} ({target.address})", file=sys.stderr)

    jsonl_path = Path(args.jsonl) if args.jsonl else None
    jsonl_file = jsonl_path.open("a", encoding="utf-8") if jsonl_path else None

    def emit(event: Dict[str, Any]) -> None:
        event.setdefault("timestamp_ms", now_ms())
        print(json.dumps(event, ensure_ascii=False))
        if jsonl_file:
            jsonl_file.write(json.dumps(event, ensure_ascii=False) + "\n")
            jsonl_file.flush()

    try:
        async with BleakClient(target.address) as client:
            emit({"event": "connected", "address": target.address, "name": target.name})

            for service in client.services:
                emit(
                    {
                        "event": "service",
                        "uuid": service.uuid,
                        "description": service.description,
                    }
                )
                for char in service.characteristics:
                    emit(
                        {
                            "event": "characteristic",
                            "service_uuid": service.uuid,
                            "uuid": char.uuid,
                            "description": char.description,
                            "properties": list(char.properties),
                        }
                    )
                    if "read" in char.properties:
                        try:
                            data = await client.read_gatt_char(char.uuid)
                            emit(
                                {
                                    "event": "read",
                                    "uuid": char.uuid,
                                    "raw_hex": hex_bytes(bytes(data)),
                                }
                            )
                        except Exception as exc:  # BLE stacks vary by OS.
                            emit({"event": "read_error", "uuid": char.uuid, "error": str(exc)})

            async def on_notify(sender: Any, data: bytearray) -> None:
                raw = bytes(data)
                emit(
                    {
                        "event": "notify",
                        "sender": str(sender),
                        "raw_hex": hex_bytes(raw),
                        "decoded": decode_modbus_register_response(raw),
                    }
                )

            notify_chars = [
                char.uuid
                for service in client.services
                for char in service.characteristics
                if "notify" in char.properties or "indicate" in char.properties
            ]
            for uuid in notify_chars:
                try:
                    await client.start_notify(uuid, on_notify)
                    emit({"event": "notify_started", "uuid": uuid})
                except Exception as exc:
                    emit({"event": "notify_error", "uuid": uuid, "error": str(exc)})

            if args.probe and await has_char(client, BLUETTI_WRITE_UUID):
                for start in args.probe_start:
                    request = read_holding_registers(start, args.probe_count)
                    emit({"event": "probe_write", "start": start, "count": args.probe_count, "raw_hex": hex_bytes(request)})
                    await client.write_gatt_char(BLUETTI_WRITE_UUID, request, response=False)
                    await asyncio.sleep(args.probe_pause)

            await asyncio.sleep(args.seconds)
    finally:
        if jsonl_file:
            jsonl_file.close()


async def has_char(client: Any, uuid: str) -> bool:
    return any(
        char.uuid.lower() == uuid.lower()
        for service in client.services
        for char in service.characteristics
    )


async def read_register_range(
    client: Any,
    queue: asyncio.Queue,
    start: int,
    count: int,
    response_timeout: float,
    function_code: int = 3,
) -> Dict[str, Any]:
    request = read_registers(start, count, function_code=function_code)
    await client.write_gatt_char(BLUETTI_WRITE_UUID, request, response=False)
    try:
        frame = await asyncio.wait_for(queue.get(), timeout=response_timeout)
    except asyncio.TimeoutError:
        return {
            "timestamp_ms": now_ms(),
            "start": start,
            "count": count,
            "event": "timeout",
            "registers_u16": [],
            "registers_by_address": {},
        }

    decoded = decode_modbus_register_response(frame)
    words = decoded.get("registers_u16", [])
    return {
        "timestamp_ms": now_ms(),
        "start": start,
        "count": count,
        "function": function_code,
        "raw_hex": decoded["raw_hex"],
        "crc_ok": decoded["crc_ok"],
        "exception": decoded.get("exception"),
        "error": decoded.get("error"),
        "registers_u16": words,
        "registers_by_address": registers_by_address(start, words),
        "heuristics": decoded.get("heuristics", {}),
    }


async def poll(args: argparse.Namespace) -> None:
    if BleakClient is None:
        raise RuntimeError("bleak is not installed. Run: python3 -m pip install bleak")
    target = await find_target(args.address, args.name, args.timeout)
    mapping = load_map(args.map)
    ranges = parse_ranges(args)
    print(f"Connecting to {target.name or target.address} ({target.address})", file=sys.stderr)

    rows: List[Dict[str, Any]] = []
    csv_file = None
    writer = None
    if args.csv:
        csv_file = Path(args.csv).open("a", newline="", encoding="utf-8")
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "timestamp_ms",
                "start",
                "count",
                "raw_hex",
                "crc_ok",
                "ranges",
                "registers_u16",
                "values",
            ],
        )
        if csv_file.tell() == 0:
            writer.writeheader()

    async with BleakClient(target.address) as client:
        if not await has_char(client, BLUETTI_NOTIFY_UUID) or not await has_char(client, BLUETTI_WRITE_UUID):
            raise RuntimeError("Bluetti ff01/ff02 characteristics not found. Run dump first.")

        queue: asyncio.Queue[bytes] = asyncio.Queue()

        def on_notify(_: Any, data: bytearray) -> None:
            queue.put_nowait(bytes(data))

        await client.start_notify(BLUETTI_NOTIFY_UUID, on_notify)

        for _ in range(args.samples):
            range_rows = []
            merged_registers: Dict[str, int] = {}
            flat_words = []
            for start, count in ranges:
                range_row = await read_register_range(client, queue, start, count, args.response_timeout, args.function)
                range_rows.append(range_row)
                merged_registers.update(range_row["registers_by_address"])
                flat_words.extend(range_row["registers_u16"])
                await asyncio.sleep(args.range_pause)

            parsed = parse_register_map(merged_registers, mapping)
            row = {
                "timestamp_ms": now_ms(),
                "ranges": range_rows,
                "registers_u16": flat_words,
                "registers_by_address": merged_registers,
                "values": parsed,
            }
            rows.append(row)
            print(json.dumps(row, ensure_ascii=False))
            if writer:
                writer.writerow(
                    {
                        "timestamp_ms": row["timestamp_ms"],
                        "start": ranges[0][0],
                        "count": ranges[0][1],
                        "raw_hex": json.dumps([r.get("raw_hex") for r in range_rows]),
                        "crc_ok": all(r.get("crc_ok") for r in range_rows),
                        "ranges": json.dumps(range_rows),
                        "registers_u16": json.dumps(row["registers_u16"]),
                        "values": json.dumps(row["values"]),
                    }
                )
                csv_file.flush()
            await asyncio.sleep(args.interval)

    if csv_file:
        csv_file.close()


async def discover(args: argparse.Namespace) -> None:
    if BleakClient is None:
        raise RuntimeError("bleak is not installed. Run: python3 -m pip install bleak")
    target = await find_target(args.address, args.name, args.timeout)
    print(f"Connecting to {target.name or target.address} ({target.address})", file=sys.stderr)
    print(
        f"Scanning registers {args.start}..{args.end} with function {args.function}",
        file=sys.stderr,
    )

    async with BleakClient(target.address) as client:
        if not await has_char(client, BLUETTI_NOTIFY_UUID) or not await has_char(client, BLUETTI_WRITE_UUID):
            raise RuntimeError("Bluetti ff01/ff02 characteristics not found. Run dump first.")

        queue: asyncio.Queue[bytes] = asyncio.Queue()

        def on_notify(_: Any, data: bytearray) -> None:
            queue.put_nowait(bytes(data))

        await client.start_notify(BLUETTI_NOTIFY_UUID, on_notify)

        found: Dict[str, int] = {}
        for address in range(args.start, args.end + 1):
            row = await read_register_range(
                client,
                queue,
                address,
                1,
                args.response_timeout,
                args.function,
            )
            if row.get("registers_by_address"):
                found.update(row["registers_by_address"])
                print(json.dumps({"timestamp_ms": now_ms(), "found": row}, ensure_ascii=False))
            elif args.show_errors:
                print(json.dumps({"timestamp_ms": now_ms(), "miss": row}, ensure_ascii=False))
            await asyncio.sleep(args.pause)

        print(json.dumps({"timestamp_ms": now_ms(), "found_registers": found}, ensure_ascii=False))


def mqtt_connect(args: argparse.Namespace) -> Any:
    if mqtt is None:
        raise RuntimeError("paho-mqtt is not installed. Run: python3 -m pip install paho-mqtt")

    client = make_mqtt_client(args.client_id or "")
    if args.username:
        client.username_pw_set(args.username, args.password)
    client.connect(args.broker, args.port, keepalive=60)
    client.loop_start()
    return client


def make_mqtt_client(client_id: str) -> Any:
    callback_api_version = getattr(getattr(mqtt, "CallbackAPIVersion", None), "VERSION2", None)
    if callback_api_version is not None:
        return mqtt.Client(callback_api_version=callback_api_version, client_id=client_id)
    return mqtt.Client(client_id=client_id)


def mqtt_publish_json(client: Any, topic: str, payload: Dict[str, Any], retain: bool) -> None:
    client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=0, retain=retain)


async def mqtt_loop(args: argparse.Namespace) -> None:
    if BleakClient is None:
        raise RuntimeError("bleak is not installed. Run: python3 -m pip install bleak")
    target = await find_target(args.address, args.name, args.timeout)
    mapping = load_map(args.map)
    ranges = parse_ranges(args)
    topic_base = args.topic.rstrip("/")
    client = mqtt_connect(args)
    print(f"Connecting to {target.name or target.address} ({target.address})", file=sys.stderr)
    print(f"Publishing to mqtt://{args.broker}:{args.port}/{topic_base}/...", file=sys.stderr)

    try:
        async with BleakClient(target.address) as ble_client:
            if not await has_char(ble_client, BLUETTI_NOTIFY_UUID) or not await has_char(ble_client, BLUETTI_WRITE_UUID):
                raise RuntimeError("Bluetti ff01/ff02 characteristics not found. Run dump first.")

            queue: asyncio.Queue[bytes] = asyncio.Queue()

            def on_notify(_: Any, data: bytearray) -> None:
                queue.put_nowait(bytes(data))

            await ble_client.start_notify(BLUETTI_NOTIFY_UUID, on_notify)
            mqtt_publish_json(
                client,
                f"{topic_base}/status",
                {"online": True, "address": target.address, "name": target.name, "timestamp_ms": now_ms()},
                retain=True,
            )

            sample = 0
            while args.samples == 0 or sample < args.samples:
                range_rows = []
                merged_registers: Dict[str, int] = {}
                flat_words = []
                for start, count in ranges:
                    range_row = await read_register_range(ble_client, queue, start, count, args.response_timeout, args.function)
                    range_rows.append(range_row)
                    merged_registers.update(range_row["registers_by_address"])
                    flat_words.extend(range_row["registers_u16"])
                    await asyncio.sleep(args.range_pause)

                parsed = parse_register_map(merged_registers, mapping)
                payload = {
                    "timestamp_ms": now_ms(),
                    "ranges": range_rows,
                    "crc_ok": all(row.get("crc_ok") for row in range_rows),
                    "registers_u16": flat_words,
                    "registers_by_address": merged_registers,
                    "values": parsed,
                }

                print(json.dumps(payload, ensure_ascii=False))
                mqtt_publish_json(client, f"{topic_base}/raw", payload, retain=args.retain)
                mqtt_publish_json(
                    client,
                    f"{topic_base}/registers",
                    {
                        "timestamp_ms": payload["timestamp_ms"],
                        "registers_by_address": merged_registers,
                        "registers_u16": flat_words,
                    },
                    retain=args.retain,
                )
                mqtt_publish_json(
                    client,
                    f"{topic_base}/values",
                    {
                        "timestamp_ms": payload["timestamp_ms"],
                        "values": parsed,
                    },
                    retain=args.retain,
                )
                for name, item in parsed.items():
                    mqtt_publish_json(
                        client,
                        f"{topic_base}/value/{name}",
                        {
                            "timestamp_ms": payload["timestamp_ms"],
                            **item,
                        },
                        retain=args.retain,
                    )

                sample += 1
                await asyncio.sleep(args.interval)
    finally:
        try:
            mqtt_publish_json(client, f"{topic_base}/status", {"online": False, "timestamp_ms": now_ms()}, retain=True)
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bluetti AC180 BLE reader")
    sub = parser.add_subparsers(dest="command", required=True)

    scan_parser = sub.add_parser("scan", help="Scan BLE devices and print JSON")
    scan_parser.add_argument("--timeout", type=float, default=8.0)
    scan_parser.set_defaults(func=scan)

    dump_parser = sub.add_parser("dump", help="Discover services, read characteristics, and capture notifications")
    add_target_args(dump_parser)
    dump_parser.add_argument("--seconds", type=float, default=20.0)
    dump_parser.add_argument("--jsonl", help="Append raw events to a JSONL file")
    dump_parser.add_argument("--probe", action="store_true", help="Send common Bluetti Modbus register-read probes")
    dump_parser.add_argument("--probe-start", type=int, nargs="+", default=[0, 100, 200, 300])
    dump_parser.add_argument("--probe-count", type=int, default=60)
    dump_parser.add_argument("--probe-pause", type=float, default=1.0)
    dump_parser.set_defaults(func=dump)

    poll_parser = sub.add_parser("poll", help="Poll one or more Modbus register ranges repeatedly")
    add_target_args(poll_parser)
    poll_parser.add_argument("--start", type=int, default=0)
    poll_parser.add_argument("--count", type=int, default=80)
    poll_parser.add_argument("--ranges", nargs="+", help="One or more START:COUNT ranges, for example 36:14 70:24")
    poll_parser.add_argument("--range-pause", type=float, default=0.2, help="Pause between multi-range reads")
    poll_parser.add_argument("--function", type=int, choices=[3, 4], default=3, help="Modbus function: 3 holding registers, 4 input registers")
    poll_parser.add_argument("--samples", type=int, default=10)
    poll_parser.add_argument("--interval", type=float, default=2.0)
    poll_parser.add_argument("--response-timeout", type=float, default=4.0)
    poll_parser.add_argument("--map", help="JSON register map file")
    poll_parser.add_argument("--csv", help="Append decoded register rows to CSV")
    poll_parser.set_defaults(func=poll)

    discover_parser = sub.add_parser("discover", help="Scan register addresses one by one")
    add_target_args(discover_parser)
    discover_parser.add_argument("--start", type=int, default=0)
    discover_parser.add_argument("--end", type=int, default=300)
    discover_parser.add_argument("--function", type=int, choices=[3, 4], default=3, help="Modbus function: 3 holding registers, 4 input registers")
    discover_parser.add_argument("--pause", type=float, default=0.05)
    discover_parser.add_argument("--response-timeout", type=float, default=2.0)
    discover_parser.add_argument("--show-errors", action="store_true")
    discover_parser.set_defaults(func=discover)

    mqtt_parser = sub.add_parser("mqtt", help="Poll registers and publish JSON to an MQTT broker")
    add_target_args(mqtt_parser)
    mqtt_parser.add_argument("--broker", required=True, help="MQTT broker host/IP")
    mqtt_parser.add_argument("--port", type=int, default=1883)
    mqtt_parser.add_argument("--username")
    mqtt_parser.add_argument("--password")
    mqtt_parser.add_argument("--client-id", default="bluetti-ac180")
    mqtt_parser.add_argument("--topic", default="bluetti/ac180")
    mqtt_parser.add_argument("--retain", action="store_true", help="Publish telemetry with MQTT retain flag")
    mqtt_parser.add_argument("--map", help="JSON register map file")
    mqtt_parser.add_argument("--start", type=int, default=0)
    mqtt_parser.add_argument("--count", type=int, default=80)
    mqtt_parser.add_argument("--ranges", nargs="+", help="One or more START:COUNT ranges, for example 36:14 70:24")
    mqtt_parser.add_argument("--range-pause", type=float, default=0.2, help="Pause between multi-range reads")
    mqtt_parser.add_argument("--function", type=int, choices=[3, 4], default=3, help="Modbus function: 3 holding registers, 4 input registers")
    mqtt_parser.add_argument("--samples", type=int, default=0, help="0 means run forever")
    mqtt_parser.add_argument("--interval", type=float, default=5.0)
    mqtt_parser.add_argument("--response-timeout", type=float, default=4.0)
    mqtt_parser.set_defaults(func=mqtt_loop)

    return parser


def add_target_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--address", help="BLE MAC/address/UUID from scan output")
    parser.add_argument("--name", default="AC180", help="Substring to match in BLE advertisement name")
    parser.add_argument("--timeout", type=float, default=8.0, help="Discovery timeout when --address is omitted")


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    try:
        asyncio.run(args.func(args))
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
