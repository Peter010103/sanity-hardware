#!/usr/bin/env python3
"""Build a separate communications or flight image; never flashes hardware."""
import argparse
import hashlib
import ipaddress
import json
from pathlib import Path
import re
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--board',choices=('xiao-s3','stamp-p4'),default='xiao-s3')
    parser.add_argument('--backend',choices=('zenoh','udp-unicast'),default='zenoh')
    parser.add_argument('--diagnostic-level',choices=('full','minimal','control-only'),default='minimal')
    parser.add_argument('--profile',choices=('tcp-shared','tcp-split','udp-split','udp-shared'),required=True)
    parser.add_argument('--base-config',type=Path,required=True,help='Private sdkconfig with existing Wi-Fi/wiring settings; never printed')
    parser.add_argument('--idf-path',type=Path,required=True)
    parser.add_argument('--pico-source',type=Path,required=True)
    parser.add_argument('--host-ip',default='10.0.0.200')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--enable-tuning',action='store_true',help='Enable bounded MSP tuning RPC in a flight image')
    parser.add_argument('--flight',action='store_true',help='Build flight-capable firmware: remove the disarmed-only test guard and telemetry toggle')
    parser.add_argument('--optimize',action='store_true',help='Use240MHz and performance compiler optimization in this test build')
    parser.add_argument('--command-timing',action='store_true',help='Optional schema1 per-command decision/UART enqueue trace; measurement overhead requires bench verification')
    parser.add_argument('--estimator',action='store_true',help='921600 8N1 observation ESKF experiment; requires --mocap and shared Zenoh, disarmed-only unless an explicit hover profile is selected')
    hover=parser.add_mutually_exclusive_group()
    hover.add_argument('--estimator-pid-hover',action='store_true',help='Explicit PID-only flight with ESKF logging; requires --flight --estimator --mocap, and rejects INDI/direct commands')
    hover.add_argument('--estimator-indi-hover',action='store_true',help='Explicit INDI-capable flight with ESKF logging; requires --flight --estimator --mocap, and accepts only SZC4 CTBR controller selection')
    parser.add_argument('--paper-native',action='store_true',help='Selectable PIN1 native paper transport with observation ESKF; requires --flight --estimator --mocap --profile tcp-shared')
    parser.add_argument('--mocap',action='store_true',help='Enable observation-only calibrated mocap state cache and clock exchange')
    parser.add_argument('--hx711',action='store_true',help='Optional raw HX711 UDP telemetry: D0/GPIO1 data, D1/GPIO2 clock')
    parser.add_argument('--hx711-profile',choices=('bench','flight'),help='Explicit HX711 safety profile; flight also requires --flight')
    parser.add_argument('--hx711-port',type=int,default=7460)
    args=parser.parse_args();
    estimator_hover=args.estimator_pid_hover or args.estimator_indi_hover or args.paper_native
    if args.paper_native and (args.estimator_pid_hover or args.estimator_indi_hover or args.profile!='tcp-shared'):parser.error('--paper-native requires tcp-shared and no legacy hover profile')
    if estimator_hover and (not args.flight or not args.estimator or args.enable_tuning or args.hx711):parser.error('Estimator hover profiles require --flight --estimator, without tuning or HX711')
    if args.estimator and (args.flight != estimator_hover or not args.mocap or args.backend!='zenoh' or not args.profile.endswith('shared')):parser.error('--estimator requires --mocap, shared Zenoh, and either a disarmed build or an explicit --flight estimator hover profile')
    if args.hx711_profile and not args.hx711:parser.error('--hx711-profile requires --hx711')
    if args.hx711 and args.flight and args.hx711_profile!='flight':parser.error('HX711 flight builds require explicit --hx711-profile flight; do not edit the bench guard')
    if args.hx711_profile=='flight' and not args.flight:parser.error('HX711 flight profile requires --flight')
    if args.hx711_profile=='bench' and args.flight:parser.error('Bench HX711 profile must retain disarmed-only guard')
    if not 1024<=args.hx711_port<=65535:parser.error('HX711 port must be 1024..65535')
    address=ipaddress.IPv4Address(args.host_ip)
    if address.is_unspecified or address.is_multicast or str(address)=="255.255.255.255":parser.error("Explicit unicast host IPv4 required")
    if args.enable_tuning and args.backend=='udp-unicast':parser.error('MSP tuning RPC is not part of the UDP baseline')
    if args.mocap and args.backend=='udp-unicast':parser.error('--mocap requires the Zenoh backend')
    if args.enable_tuning and not args.flight:parser.error("--enable-tuning requires --flight")
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False,mode=0o700)
    project=Path(__file__).resolve().parents[1]
    settings={'CONFIG_SANITY_DATAGRAM_HOST':json.dumps(args.host_ip),'CONFIG_SANITY_DATAGRAM_PORT':'7451','CONFIG_SANITY_DATAGRAM_HOST_PORT':'7450','CONFIG_SANITY_RAW_UDP':'y' if args.backend=='udp-unicast' else 'n','CONFIG_SANITY_DIAGNOSTIC_LEVEL':str(('full','minimal','control-only').index(args.diagnostic_level)),'CONFIG_SANITY_MSP_TUNING':'y' if args.enable_tuning else 'n','CONFIG_SANITY_COMMS_TEST_ONLY':'n' if args.flight else 'y','CONFIG_SANITY_TIMING_DIAGNOSTICS':'y',
        'CONFIG_SANITY_SPLIT_TELEMETRY':'n' if args.profile.endswith('shared') else 'y',
        'CONFIG_SANITY_ZENOH_ENDPOINT':json.dumps(('udp' if args.profile.startswith('udp') else 'tcp')+'/'+args.host_ip+':7447'),
        'CONFIG_SANITY_TELEMETRY_ENDPOINT':json.dumps('tcp/'+args.host_ip+':7448')}
    settings.update({'CONFIG_SANITY_HX711':'y' if args.hx711 else 'n',
        'CONFIG_SANITY_HX711_DOUT':'1','CONFIG_SANITY_HX711_SCK':'2',
        'CONFIG_SANITY_HX711_HOST':json.dumps(args.host_ip),'CONFIG_SANITY_HX711_PORT':str(args.hx711_port)})
    settings['CONFIG_SANITY_COMMAND_TIMING_TRACE']='y' if args.command_timing else 'n'
    if args.estimator:settings['CONFIG_FREERTOS_HZ']='1000'
    settings['CONFIG_SANITY_PAPER_NATIVE']='y' if args.paper_native else 'n'
    settings['CONFIG_SANITY_ESTIMATOR_EXPERIMENT']='y' if args.estimator else 'n'
    settings['CONFIG_SANITY_ESTIMATOR_PID_HOVER']='y' if args.estimator_pid_hover else 'n'
    settings['CONFIG_SANITY_ESTIMATOR_INDI_HOVER']='y' if args.estimator_indi_hover else 'n'
    settings['CONFIG_SANITY_MOCAP_STREAM']='y' if args.mocap else 'n'
    hx_profile=(args.hx711_profile or 'bench') if args.hx711 else None
    if hx_profile:
        preset=Path(__file__).resolve().parents[1]/'profiles'/('hx711_'+hx_profile+'.defaults')
        for line in preset.read_text().splitlines():
            match=re.fullmatch(r'(CONFIG_[A-Z0-9_]+)=(.*)',line)
            unset=re.fullmatch(r'# (CONFIG_[A-Z0-9_]+) is not set',line)
            if match:settings[match[1]]=match[2]
            elif unset:settings[unset[1]]='n'
    if args.board=='stamp-p4' and args.hx711:
        parser.error('P4 HX711 timing is not qualified; use the XIAO sensor profile')
    if args.optimize and args.board=='xiao-s3':
        settings.update({'CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240':'y','CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_160':'n','CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_80':'n',
            'CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ':'240','CONFIG_COMPILER_OPTIMIZATION_PERF':'y','CONFIG_COMPILER_OPTIMIZATION_DEBUG':'n',
            'CONFIG_COMPILER_OPTIMIZATION_SIZE':'n','CONFIG_COMPILER_OPTIMIZATION_NONE':'n'})
    patch=project/'patches/zenoh-pico-1.10.1-transport-lifetime.patch'
    guard=subprocess.run(['git','apply','--reverse','--check',str(patch.resolve())],cwd=args.pico_source,capture_output=True)
    if guard.returncode:raise SystemExit('Prepare a private Pico checkout with scripts/prepare_pico.py; transport lifetime patch required')
    text=args.base_config.read_text()
    target='esp32p4' if args.board=='stamp-p4' else 'esp32s3'
    if args.board=='stamp-p4':
        # Retain receiver identity/network/protocol settings, never S3 SoC options.
        text='\n'.join(line for line in text.splitlines()
                       if re.match(r'(?:# )?CONFIG_SANITY_',line))
        text+='\n'+(project/'sdkconfig.defaults').read_text()
        for line in (project/'profiles/stamp_p4_c6.defaults').read_text().splitlines():
            match=re.fullmatch(r'(CONFIG_[A-Z0-9_]+)=(.*)',line)
            unset=re.fullmatch(r'# (CONFIG_[A-Z0-9_]+) is not set',line)
            if match:settings[match[1]]=match[2]
            elif unset:settings[unset[1]]='n'
    lines=[]
    for line in text.splitlines():
        match=re.match(r'(?:# )?(CONFIG_[A-Z0-9_]+)(?:=| is not set)',line)
        if match and match[1] in settings:continue
        lines.append(line)
    lines.extend((key+'='+value if value!='n' else '# '+key+' is not set') for key,value in settings.items())
    config=out/'sdkconfig';config.write_text('\n'.join(lines)+'\n');config.chmod(0o600)
    command=['bash','-c','set -e; source "$1/export.sh" >/dev/null; shift; exec python "$@"','sanity-build',str(args.idf_path.resolve()),str(args.idf_path.resolve()/'tools/idf.py'),'-C',str(project),'-B',str(out),'-D','IDF_TARGET='+target,'-D','SDKCONFIG='+str(config),'-D','ZENOH_PICO_SOURCE='+str(args.pico_source.resolve()),'build']
    with (out/'build.log').open('w') as log:result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
    if result.returncode:raise SystemExit('Build failed; see '+str(out/'build.log'))
    image=out/'sanity_zenoh_receiver.bin'
    if not image.exists():
        candidates=list(out.glob('*.bin'))
        if len(candidates)!=1:raise SystemExit('Cannot identify application binary')
        image=candidates[0]
    resolved=json.loads((out/'config/sdkconfig.json').read_text())
    if resolved.get('IDF_TARGET')!=target:
        raise SystemExit('Requested board target did not resolve')
    if args.board=='stamp-p4':
        expected={'SANITY_UART_TX':10,'SANITY_UART_RX':12,
                  'SANITY_PASSTHROUGH_BUTTON_GPIO':35,'SANITY_PASSTHROUGH_LED_GPIO':-1,
                  'ESP_HOSTED_SDIO_GPIO_RESET_SLAVE':42,'ESP_HOSTED_SDIO_PIN_CLK':43,
                  'ESP_HOSTED_SDIO_PIN_CMD':44,'ESP_HOSTED_SDIO_PIN_D0':45,
                  'ESP_HOSTED_SDIO_PIN_D1':46,'ESP_HOSTED_SDIO_PIN_D2':47,
                  'ESP_HOSTED_SDIO_PIN_D3':48,'ESP32P4_REV_MIN_FULL':100}
        if any(resolved.get(key)!=value for key,value in expected.items()) or not resolved.get('ESP_WIFI_REMOTE_ENABLED'):
            raise SystemExit('P4 board configuration did not resolve; refusing an unverified pin map')
    if resolved.get('SANITY_COMMAND_TIMING_TRACE',False)!=args.command_timing:
        raise SystemExit('Requested command-timing configuration did not resolve')
    if args.estimator:
        required=('SANITY_ESTIMATOR_EXPERIMENT','SANITY_MOCAP_STREAM','SANITY_MSP_UNIFIED')
        forbidden=('SANITY_SPLIT_TELEMETRY','SANITY_RAW_UDP') + (('SANITY_MSP_TUNING','SANITY_HX711') if estimator_hover else ())
        if not all(resolved.get(key,False) for key in required) or resolved.get('SANITY_COMMS_TEST_ONLY',False)==estimator_hover or resolved.get('SANITY_ESTIMATOR_PID_HOVER',False)!=args.estimator_pid_hover or resolved.get('SANITY_ESTIMATOR_INDI_HOVER',False)!=args.estimator_indi_hover or resolved.get('FREERTOS_HZ')!=1000 or any(resolved.get(key,False) for key in forbidden):
            raise SystemExit('Requested estimator configuration did not resolve; inspect private SDK configuration locally')
    if resolved.get('SANITY_PAPER_NATIVE',False)!=args.paper_native:raise SystemExit('Requested paper native configuration did not resolve')
    if args.hx711:
        assert resolved['SANITY_HX711'] and resolved['SANITY_COMMS_TEST_ONLY']==(hx_profile=='bench')
        assert not resolved.get('PM_ENABLE',False) and resolved['GPIO_CTRL_FUNC_IN_IRAM']
        assert resolved['ESP_DEFAULT_CPU_FREQ_MHZ']==240 and resolved['SANITY_MSP_UNIFIED']
    receipt={'board':args.board,'target':target,'uart_tx_gpio':resolved['SANITY_UART_TX'],'uart_rx_gpio':resolved['SANITY_UART_RX'],'reserved_serial_rx_gpio':8 if args.board=='stamp-p4' else None,'hx711_profile':hx_profile,'intended_use':('disarmed-bench' if hx_profile=='bench' else 'flight-compatible-unqualified') if args.hx711 else 'receiver',
        'qualification':'build-only; hardware workload not verified','hx711_timing_extension':1 if args.hx711 else None,'backend':args.backend,'diagnostic_level':args.diagnostic_level,'profile':args.profile,'test_only':resolved['SANITY_COMMS_TEST_ONLY'],'timing_diagnostics':resolved['SANITY_TIMING_DIAGNOSTICS'],
        'hx711_enabled':resolved.get('SANITY_HX711',False),'hx711_port':resolved.get('SANITY_HX711_PORT'),
        'MSP_tuning':resolved.get('SANITY_MSP_TUNING',False),'split_telemetry':resolved.get('SANITY_SPLIT_TELEMETRY',False),'optimized':args.optimize,'flight_requested':args.flight,
        'CPU_MHz':resolved['ESP_DEFAULT_CPU_FREQ_MHZ'],'performance_compiler':resolved['COMPILER_OPTIMIZATION_PERF'],
        'paper_native':resolved.get('SANITY_PAPER_NATIVE',False),'estimator_experiment':resolved.get('SANITY_ESTIMATOR_EXPERIMENT',False),'estimator_pid_hover':resolved.get('SANITY_ESTIMATOR_PID_HOVER',False),'estimator_indi_hover':resolved.get('SANITY_ESTIMATOR_INDI_HOVER',False),'estimator_controls_aircraft':False,'freertos_hz':resolved.get('FREERTOS_HZ'),'effective_uart_baud':921600 if resolved.get('SANITY_ESTIMATOR_EXPERIMENT',False) else resolved.get('SANITY_MSP_BAUD'),
        'mocap_stream':resolved.get('SANITY_MOCAP_STREAM',False),'command_timing_schema':1 if resolved.get('SANITY_COMMAND_TIMING_TRACE',False) else None,
        'transport_lifetime_guard':True,'application':str(image),'bytes':image.stat().st_size,'sha256':hashlib.sha256(image.read_bytes()).hexdigest(),'flashed':False,
        'note':'Private SDK config retained in restricted output directory; no Wi-Fi credentials in this receipt.'}
    (out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt,indent=2))


if __name__=='__main__':main()
