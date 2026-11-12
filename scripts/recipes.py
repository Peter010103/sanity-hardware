"""Named experiment recipes; reference speed is distinct from measured speed."""
RECIPES = {
 'pid-circle-smoke': dict(family='legacy', pattern='circle', speed=.3, radius=.5, laps=2, evidence='low-speed commissioning reference'),
 'pid-circle-1p5': dict(family='legacy', pattern='circle', speed=1.5, radius=1., laps=5, evidence='completed physical flight, 22 October'),
 'pid-lemniscate-1p5': dict(family='legacy', pattern='lemniscate', speed=1.5, radius=3.2, laps=5, evidence='completed physical flight, 22 October'),
 'pid-circle-2p2': dict(family='legacy', pattern='circle', speed=2.2, radius=1.5, laps=5, evidence='completed physical flight, 22 October'),
 'pid-circle-3': dict(family='legacy', pattern='circle', speed=3., radius=2., laps=5, evidence='completed physical flight, 22 October'),
 'pid-circle-5': dict(family='legacy', pattern='circle', speed=5., radius=2., laps=5, high_bank=True, evidence='two completed physical references; measured cruise about4.28m/s'),
 'pid-flip-1': dict(family='pitch', laps=1, evidence='completed one inversion, 22 October'),
 'pid-flip-10': dict(family='pitch', laps=10, evidence='two completed ten-inversion flights, 22 October'),
 'paired-pid-0p8': dict(family='benchmark', inner='pid', speed=.8, radius=3.2, schedule='constant_arclength', evidence='matched-comparison recipe; see results for exact outcomes'),
 'indi-lemniscate-1p5': dict(family='benchmark', inner='indi', speed=1.5, radius=3.2, schedule='indi_shaped_angular', evidence='identified predictor and C3 transfers; mocap coverage depends on arena'),
 'indi-lemniscate-r2-1p5': dict(family='benchmark', inner='indi', speed=1.5, radius=2., schedule='indi_shaped_angular', evidence='completed physical flight,12November'),
 'indi-lemniscate-r2-3': dict(family='benchmark', inner='indi', speed=3., radius=2., schedule='indi_shaped_angular', evidence='completed physical flight after earlier interrupted attempts,12November'),
 'indi-lemniscate-r2-4': dict(family='high-speed', inner='indi', speed=4., radius=2., schedule='indi_shaped_angular', evidence='one completed physical flight after an envelope stop,12November'),
 'indi-lemniscate-r2-5': dict(family='high-speed', inner='indi', speed=5., radius=2., schedule='indi_shaped_angular', evidence='physical attempts stopped on envelope; not a completed flight'),
 'indi-lemniscate-r2-6': dict(family='high-speed', inner='indi', speed=6., radius=2., schedule='indi_shaped_angular', evidence='prepared/simulated reference; no completed physical evidence included'),
}

# November12 matched 4x3m, alternating inclined-plane reference: three cycles/six eights.
for inner,speeds in [('indi',(3,4,5)),('pid',(3,5))]:
    for speed in speeds:
        RECIPES[f'{inner}-butterfly-{speed}']=dict(family='butterfly-'+inner,inner=inner,speed=float(speed),radius=2.,schedule='indi_shaped_angular',
            evidence='completed six-eight physical flight,12November' if speed in (3,4) else 'interrupted physical attempt; see paired report',
            width_m=3.,butterfly_height_m=.3,butterfly_cycles=3)
