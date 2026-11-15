"""Read-only UDP discovery, independent of the active video/control connections."""
import ipaddress
import json
import os
import secrets
import socket
import subprocess
import time

from wire import DISCOVERY_PORT,DISCOVERY_QUERY,DISCOVERY_RESPONSE


def broadcast_targets(known_hosts=()):
    hosts={'255.255.255.255'}
    if os.name=='nt':
        command="[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new(); Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.AddressState -eq 'Preferred' } | Select-Object IPAddress,PrefixLength | ConvertTo-Json -Compress"
        try:
            result=subprocess.run(['powershell.exe','-NoProfile','-Command',command],capture_output=True,
                                  text=True,encoding='utf-8-sig',timeout=3,
                                  creationflags=subprocess.CREATE_NO_WINDOW)
            values=json.loads(result.stdout) if result.returncode==0 and result.stdout.strip() else []
            for adapter in values if isinstance(values,list) else [values]:
                address=ipaddress.IPv4Address(adapter['IPAddress'])
                prefix=int(adapter['PrefixLength'])
                if address.is_loopback or address.is_unspecified or not 0<prefix<31:continue
                hosts.add(str(ipaddress.IPv4Network((address,prefix),strict=False).broadcast_address))
        except (OSError,ValueError,KeyError,TypeError,subprocess.TimeoutExpired):pass
    for host in known_hosts:
        try:hosts.add(str(ipaddress.IPv4Address(host)))
        except ipaddress.AddressValueError:continue
    return [(host,DISCOVERY_PORT) for host in sorted(hosts)]


def parse_device(payload,address,nonce):
    try:
        value=json.loads(payload)
        if value.get('protocol')!=DISCOVERY_RESPONSE or value.get('nonce')!=nonce:return None
        if value.get('role') not in ('material','qr'):return None
        for key in ('control_port','video_port'):
            if type(value.get(key)) is not int or not 1<=value[key]<=65535:return None
        host=str(ipaddress.IPv4Address(address[0]))
        name=value.get('name','MaixCAM');build=value.get('build','')
        if not isinstance(name,str) or not isinstance(build,str):return None
        return {'host':host,'name':name[:64],'build':build[:80],'role':value['role'],
                'control_port':value['control_port'],'video_port':value['video_port']}
    except (ValueError,TypeError,AttributeError):return None


def scan_devices(known_hosts=(),cancel=None,timeout=2.,targets=None):
    targets=broadcast_targets(known_hosts) if targets is None else targets
    if cancel is not None and cancel.is_set():return []
    nonce=secrets.token_hex(8)
    found={}
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
        sock.bind(('',0));sock.settimeout(.1)
        started=time.perf_counter();due=started
        while time.perf_counter()-started<timeout:
            if cancel is not None and cancel.is_set():break
            now=time.perf_counter()
            if now>=due:
                for target in targets:
                    try:sock.sendto(DISCOVERY_QUERY+nonce.encode('ascii'),target)
                    except OSError:continue
                due=now+.6
            try:payload,address=sock.recvfrom(2048)
            except socket.timeout:continue
            device=parse_device(payload,address,nonce)
            if device is not None:found[(device['role'],device['host'],device['control_port'])]=device
    return sorted(found.values(),key=lambda item:(item['role'],item['host']))
