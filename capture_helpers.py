"""Optional onboard capture helper, not connected to the dashboard recorder.

Requires explicit verification for each device, firmware, package and property
list. Call with asynchronous ODrive devices in their owning event loop; the
dashboard's synchronous hardware objects cannot be passed here. This helper
does not commission, arm, start, stop or change motor setpoints. Its software
triggers do not synchronize the two drives.
"""
import asyncio
import csv
import importlib.metadata
import json
import math
import time
from pathlib import Path


def validate_capabilities(properties_by_role, metadata):
    """Reject unverified channels before importing ODrive or touching a device."""
    if set(properties_by_role) != {'test', 'load'}:
        raise ValueError('Provide test and load property lists.')
    capabilities = metadata.get('capture_capabilities', {})
    for role, properties in properties_by_role.items():
        proof = capabilities.get(role, {})
        if proof.get('verified') is not True:
            raise ValueError(f'{role}: onboard capture capability has not been verified.')
        for field in ('serial', 'firmware', 'python_package_version', 'verified_at_utc', 'evidence'):
            if not isinstance(proof.get(field), str) or not proof[field].strip():
                raise ValueError(f'{role}: capability declaration requires {field}.')
        if not isinstance(properties, list) or not properties or any(not isinstance(p, str) or not p for p in properties):
            raise ValueError(f'{role}: supply a nonempty list of verified property paths.')
        if len(set(properties)) != len(properties) or proof.get('properties') != properties:
            raise ValueError(f'{role}: requested properties must exactly match the verified list, without duplicates.')
        rate = proof.get('sample_rate_hz')
        if isinstance(rate, bool) or not isinstance(rate, (float, int)) or not math.isfinite(rate) or rate <= 0:
            raise ValueError(f'{role}: a verified finite positive sample rate is required.')
        capacity = proof.get('max_samples')
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise ValueError(f'{role}: verified buffer capacity is required.')
    return capabilities


async def capture_pair(devices, properties_by_role, output_directory, metadata):
    """Capture one finite window per drive after caller-side commissioning.

    devices: {'test': connected_async_device, 'load': connected_async_device}
    output_directory must be new; existing files are never replaced.
    metadata must contain test/configuration context and capture_capabilities.
    Each role's declaration requires verified=True, serial, firmware,
    python_package_version, properties, sample_rate_hz, max_samples,
    verified_at_utc and evidence. Verification is the caller's responsibility;
    setting a flag does not establish physical compatibility.

    The caller must serialize capture ownership, monitor motor state separately,
    and handle faults. No duplicate connection or capture may own either board.
    """
    if set(devices) != {'test', 'load'}:
        raise ValueError('Provide test and load asynchronous ODrive devices.')
    capabilities = validate_capabilities(properties_by_role, metadata)
    from odrive.utils import HighRateCapturer, TimestampFmt

    version = importlib.metadata.version('odrive')
    # Bind declarations to the connected devices before capture starts.
    for role, device in devices.items():
        proof = capabilities[role]
        if proof['python_package_version'] != version:
            raise ValueError(f'{role}: installed ODrive package differs from the verified package.')
        serial = int(await device.read('serial_number'))
        parts = [int(await device.read('fw_version_' + part)) for part in ('major', 'minor', 'revision')]
        firmware = '.'.join(str(part) for part in parts)
        if serial != int(proof['serial'], 16) or firmware != proof['firmware']:
            raise ValueError(f'{role}: device serial or firmware differs from the verified capability.')

    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=False)
    record = dict(metadata, capture_source='ODRIVE_ONBOARD', status='incomplete',
                  timestamp_unit='control cycles relative to each device trigger',
                  time_s_basis='control cycles divided by declared verified sample rate',
                  synchronized=False, properties=properties_by_role,
                  installed_python_package_version=version)
    manifest = output / 'metadata.json'
    manifest.write_text(json.dumps(record, indent=2, allow_nan=False), encoding='utf-8')
    captures = {}
    try:
        for role, device in devices.items():
            captures[role] = await HighRateCapturer.from_properties(
                device, properties_by_role[role], unsafe=False)
        await asyncio.gather(*(capture.start() for capture in captures.values()))
        record['host_trigger_request_monotonic_ns'] = time.monotonic_ns()
        await asyncio.gather(*(capture.trigger(trigger_point=0.0) for capture in captures.values()))
        await asyncio.gather(*(capture.wait(timeout=10) for capture in captures.values()))
        counts = {}
        for role, capture in captures.items():
            # Preserve cycle indices instead of inheriting a package-assumed Hz.
            data = await capture.download(return_as=dict, t_fmt=TimestampFmt.CONTROL_CYCLE)
            if 'timestamps' not in data or len({len(values) for values in data.values()}) != 1:
                raise ValueError('Capture has missing timestamps or unequal column lengths.')
            count = len(data['timestamps'])
            if not 0 < count <= capabilities[role]['max_samples']:
                raise ValueError(f'{role}: returned sample count exceeds verified capacity or is empty.')
            cycles = data.pop('timestamps')
            if any(not isinstance(t, (int, float)) or not math.isfinite(t) for t in cycles):
                raise ValueError(f'{role}: invalid capture cycle timestamps.')
            data = {'trigger_cycle': cycles,
                    'time_s': [t / capabilities[role]['sample_rate_hz'] for t in cycles], **data}
            with (output / (role + '.csv')).open('x', newline='', encoding='utf-8') as stream:
                writer = csv.writer(stream)
                writer.writerow(data.keys())
                writer.writerows(zip(*data.values()))
            counts[role] = count
        record['sample_counts'] = counts
        record['status'] = 'complete'
    except Exception as exc:
        record['error'] = str(exc)
        # The owning controller handles rig faults and capture cleanup.
        raise
    finally:
        manifest.write_text(json.dumps(record, indent=2, allow_nan=False), encoding='utf-8')
    return output
