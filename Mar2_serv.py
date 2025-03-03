import base64
import logging
from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import signal
import sys

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ImageHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        try:
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            logger.info("Received POST request. Content length: %d", content_length)
            
            image_data = json.loads(post_data.decode('utf-8'))
            image_bytes = base64.b64decode(image_data["image"])
            logger.info("Image data decoded.")
            
            filename = f"received_image_{image_data.get('timestamp', 'unknown')}.jpg"
            with open(filename, "wb") as f:
                f.write(image_bytes)
            logger.info(f"Image saved to {filename}")
            
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success"}).encode())
            logger.info("Sent success response.")
        except Exception as e:
            logger.error(f"Error handling request: {e}")
            self.send_response(400)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode())
            logger.info(f"Sent error response. Error: {e}")

    # Add a GET handler for basic connectivity testing
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write(b"Image server is running. Use POST requests to upload images.")
        logger.info("Received GET request - sent test response")

def run_server(host='0.0.0.0', port=8080):
    server_address = (host, port)
    try:
        httpd = HTTPServer(server_address, ImageHandler)
        logger.info(f"Server running on {host}:{port}")
        
        # Set up signal handler for graceful shutdown
        def signal_handler(sig, frame):
            logger.info("Shutting down server...")
            httpd.server_close()
            sys.exit(0)
            
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Run the server directly in the main thread
        httpd.serve_forever()
    except Exception as e:
        logger.error(f"Error starting server: {e}")
        sys.exit(1)

if __name__ == "__main__":
    run_server()
