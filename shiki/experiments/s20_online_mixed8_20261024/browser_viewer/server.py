#!/usr/bin/env python3
"""Loopback-only browser viewer and fixed simulation launcher. Never controls hardware."""
import argparse
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import signal
import subprocess
import threading
from urllib.parse import urlsplit

HERE = Path(__file__).resolve().parent
EXPERIMENT = HERE.parent
ROOT = EXPERIMENT.parent.parent


class Session:
    def __init__(self):
        self.process = None
        self.run = None
        self.lock = threading.Lock()

    def start(self, mode="upright"):
        if mode not in ("upright", "flip-horizontal", "stacked-width", "stacked-four"):
            raise ValueError("Unsupported simulation mode")
        with self.lock:
            if self.process and self.process.poll() is None:
                return False
            self.run = ROOT/'runs'/('browser_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            log = self.run.with_suffix('.log')
            command = [str(EXPERIMENT/'s20-contact-sim'), '--arrivals',('four20' if mode in ('upright','stacked-four') else 'one20'),'--placement-mode',('stacked-width' if mode=='stacked-four' else mode),'--size-noise','0',
                       '--planning-mode','async','--weights',str(EXPERIMENT/'perception/weights/yolo11m-seg.pt'),
                       '--pose-json',str(ROOT/'runs/observe_moved_box_attempt1/controller_snapshot.json'),
                       '--waypoints',str(EXPERIMENT/'deployment/pick_waypoints.json'),
                       '--site-urdf',str(EXPERIMENT/'deployment/site_robot.urdf'),
                       '--record-decisions','--realtime','--saved-camera-only','--browser-stream',
                       '--output',str(self.run)]
            log.parent.mkdir(parents=True,exist_ok=True)
            with log.open('wb') as output:
                self.process = subprocess.Popen(command,cwd=ROOT,stdout=output,stderr=subprocess.STDOUT,
                                                start_new_session=True,env={**os.environ,'PYTHONUNBUFFERED':'1'})
            return True

    def stop(self):
        with self.lock:
            if self.process and self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)

    def status(self):
        with self.lock:
            code = self.process.poll() if self.process else None
            state = None
            if self.run and (self.run/'web/state.json').exists():
                state = json.loads((self.run/'web/state.json').read_text())
            log_tail = ''
            if self.run and code is not None:
                with self.run.with_suffix('.log').open('rb') as log:
                    log.seek(max(0,log.seek(0,2)-4000))
                    log_tail=log.read().decode('utf8',errors='replace')
            return dict(run=self.run.name if self.run else None, active=bool(self.process and code is None),
                        exit_code=code, state=state, log_tail=log_tail)


SESSION = Session()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, value, code=200):
        self.send_data(json.dumps(value,ensure_ascii=False).encode(), 'application/json',code)

    def send_data(self, data, content_type, code=200):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/api/status':
            return self.reply(SESSION.status())
        static = {'/':'index.html','/app.js':'app.js','/vendor/three.module.js':'vendor/three.module.js',
                  '/vendor/OrbitControls.js':'vendor/OrbitControls.js'}
        if path in static:
            file = HERE/static[path]
        elif path in ('/data/scene.json','/data/geometry.bin') and SESSION.run:
            file = SESSION.run/'web'/path.rsplit('/',1)[1]
        else:
            return self.reply({'error':'Not found'},404)
        if not file.is_file():
            return self.reply({'error':'Scene not ready'},404)
        self.send_data(file.read_bytes(),mimetypes.guess_type(str(file))[0] or 'application/octet-stream')

    def do_POST(self):
        # Only our same-origin UI may launch a simulation; never accept commands or file paths.
        if self.headers.get('Origin') != f'http://{self.headers.get("Host")}' or self.headers.get('X-Viewer-Request') != '1':
            return self.reply({'error':'Same-origin request required'},403)
        if self.path in ('/api/start','/api/start-flip','/api/start-stack','/api/start-stack-four'):
            mode = {'/api/start':'upright','/api/start-flip':'flip-horizontal','/api/start-stack':'stacked-width','/api/start-stack-four':'stacked-four'}[self.path]
            return self.reply({'started':SESSION.start(mode)})
        if self.path == '/api/stop':
            SESSION.stop()
            return self.reply({'stopping':True})
        self.reply({'error':'Not found'},404)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8765)
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print(f'S20 browser viewer: http://127.0.0.1:{args.port}',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        SESSION.stop()
        server.server_close()

if __name__ == '__main__':
    main()
