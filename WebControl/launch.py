#!/usr/bin/env python3
"""
launch.py - Standalone Cross-Platform Web Server & Browser Launcher
Teensy 4.1 6-DOF Robot Arm Control Studio

Features:
- Pure standard library Python 3 (zero external pip dependencies)
- Automatic port finding (starts at 8000, tries up to 8099)
- Correct MIME type mapping for ES modules (.js -> application/javascript)
- Opens default web browser automatically to http://localhost:<port>/
- Graceful shutdown on Ctrl+C
"""

import os
import sys
import argparse
import socket
import webbrowser
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler

class CustomHTTPRequestHandler(SimpleHTTPRequestHandler):
    """Custom request handler with correct MIME types for ES6 modules and WebSerial headers."""
    
    extensions_map = {
        '': 'application/octet-stream',
        '.html': 'text/html; charset=utf-8',
        '.htm': 'text/html; charset=utf-8',
        '.css': 'text/css; charset=utf-8',
        '.js': 'application/javascript; charset=utf-8',
        '.mjs': 'application/javascript; charset=utf-8',
        '.json': 'application/json; charset=utf-8',
        '.png': 'image/png',
        '.jpg': 'image/jpeg',
        '.jpeg': 'image/jpeg',
        '.gif': 'image/gif',
        '.svg': 'image/svg+xml',
        '.ico': 'image/x-icon',
        '.wasm': 'application/wasm',
        '.stl': 'model/stl',
    }

    def end_headers(self):
        # Security & isolation headers for modern Web APIs (WebSerial, SharedArrayBuffer)
        self.send_header('Cross-Origin-Opener-Policy', 'same-origin')
        self.send_header('Cross-Origin-Embedder-Policy', 'credentialless')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        super().end_headers()

    def log_message(self, format, *args):
        # Clean console log output
        sys.stdout.write(f"[{time.strftime('%H:%M:%S')}] {format % args}\n")
        sys.stdout.flush()


def find_free_port(start_port=8000, max_port=8099, host='127.0.0.1'):
    """Finds an available TCP port on the requested interface."""
    for port in range(start_port, max_port + 1):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind((host, port))
                return port
            except OSError:
                continue
    return start_port


def get_lan_ip():
    """Finds the preferred local IPv4 address without sending application data."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(('192.0.2.1', 80))
        return probe.getsockname()[0]
    except OSError:
        return None
    finally:
        probe.close()


def main():
    parser = argparse.ArgumentParser(description='Launch Control Studio locally.')
    parser.add_argument(
        '--lan',
        action='store_true',
        help='Allow other devices on the local network to open Control Studio'
    )
    args = parser.parse_args()

    # Set current working directory to this script's directory (Control Studio)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    bind_host = '0.0.0.0' if args.lan else '127.0.0.1'
    port = find_free_port(8000, host=bind_host)
    server_address = (bind_host, port)
    url = f"http://localhost:{port}/"

    print("=" * 70)
    print("  TEENSY 4.1 CONTROL STUDIO - WEB SERVER LAUNCHER")
    print("=" * 70)
    print(f"Serving directory : {script_dir}")
    print(f"Local URL          : {url}")
    if args.lan:
        lan_ip = get_lan_ip()
        if lan_ip:
            print(f"Phone URL          : http://{lan_ip}:{port}/")
        else:
            print("Phone URL          : Check this PC's Wi-Fi IPv4 address")
        print("Network access     : Enabled on all interfaces; use only on a trusted LAN")
    print(f"Port               : {port}")
    print("-" * 70)
    print("Launching default browser... (Press Ctrl+C to stop server)")
    print("=" * 70)

    # Open browser in a separate thread after short delay
    def open_browser():
        time.sleep(0.6)
        webbrowser.open(url)

    threading.Thread(target=open_browser, daemon=True).start()

    try:
        httpd = HTTPServer(server_address, CustomHTTPRequestHandler)
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down Control Studio server... Goodbye!")
        sys.exit(0)


if __name__ == '__main__':
    main()
