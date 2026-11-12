"""CPU-only response analysis in a prestarted process; no ROS, Zenoh or aircraft I/O."""
import multiprocessing
from sanity_zenoh.rate_autotune import evaluate_rate_pulses


def analysis_loop(connection):
    connection.send(('ready',None))
    try:
        while True:
            snapshot=connection.recv()
            if snapshot is None:return
            try:connection.send(('ok',evaluate_rate_pulses(snapshot)))
            except Exception as error:connection.send(('error',str(error)))
    except (EOFError,BrokenPipeError):pass
    finally:connection.close()


class RateAnalysisWorker:
    def __init__(self):
        context=multiprocessing.get_context('spawn')
        self.connection,child=context.Pipe()
        self.process=context.Process(target=analysis_loop,args=(child,),name='sanity_rate_analysis',daemon=True)
        self.process.start();child.close()
        if not self.connection.poll(10.):self.close();raise TimeoutError('Rate-analysis process startup failed before arming')
        kind,value=self.connection.recv()
        if kind!='ready':self.close();raise RuntimeError('Rate-analysis startup failed')

    def evaluate(self,measurements):
        if not self.process.is_alive():raise RuntimeError('Rate-analysis process exited')
        self.connection.send(measurements.snapshot())
        if not self.connection.poll(3.):raise TimeoutError('Rate-analysis process timed out')
        kind,value=self.connection.recv()
        if kind!='ok':raise ValueError('Rate analysis failed: '+value)
        return value

    def close(self):
        if self.process.is_alive():
            try:self.connection.send(None)
            except (OSError,EOFError):pass
            self.process.join(timeout=.2)
            if self.process.is_alive():self.process.terminate();self.process.join(timeout=.5)
        self.connection.close()
