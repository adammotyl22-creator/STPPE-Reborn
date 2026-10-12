import os
import sys
import socket
import urllib.request
import threading
import time
from http.server import HTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from launch import CustomHTTPRequestHandler, find_free_port

def test_server():
    webcontrol_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(webcontrol_dir)
    port = find_free_port(8850, 8900)
    server = HTTPServer(('127.0.0.1', port), CustomHTTPRequestHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.5)

    base_url = f"http://127.0.0.1:{port}"
    print(f"Testing server on {base_url}...")

    # 1. Fetch index.html
    req = urllib.request.urlopen(f"{base_url}/index.html")
    assert req.status == 200, "Failed to fetch index.html"
    content = req.read().decode('utf-8')
    assert "Motion control" in content, "index.html missing Motion control"
    assert "data-mode-tab" in content, "index.html missing mode tabs"
    print("[OK] index.html served successfully with 200 OK")

    # 2. Fetch main.css
    req_css = urllib.request.urlopen(f"{base_url}/css/main.css")
    assert req_css.status == 200, "Failed to fetch main.css"
    content_css = req_css.read().decode('utf-8')
    assert "--bg-void" in content_css or "cyberpunk-dark.css" in content_css, "main.css invalid"
    print("[OK] main.css served successfully with 200 OK")

    # 3. Fetch app.js
    req_js = urllib.request.urlopen(f"{base_url}/js/app.js")
    assert req_js.status == 200, "Failed to fetch js/app.js"
    print("[OK] js/app.js served successfully with 200 OK")

    server.shutdown()
    print("[OK] All server endpoints validated successfully!")

if __name__ == '__main__':
    test_server()
