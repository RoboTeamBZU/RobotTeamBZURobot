from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import cv2
import time

app = Flask(__name__)

# -----------------------------
# Camera Setup
# -----------------------------
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()
time.sleep(2)

# -----------------------------
# Video Stream Generator
# -----------------------------
def gen_frames():
    while True:
        frame = picam2.capture_array()
        
        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Encode to JPEG
        ret, buffer = cv2.imencode(".jpg", frame_bgr)
        if not ret:
            continue
        
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# -----------------------------
# Tailwind Frontend Template
# -----------------------------
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PiVision - Camera Stream</title>
<script src="https://cdn.tailwindcss.com"></script>
<script src="https://unpkg.com/feather-icons"></script>
</head>
<body class="bg-gray-900 text-gray-100 min-h-screen">
<main class="container mx-auto px-4 py-8">
    <div class="max-w-4xl mx-auto">
        <h1 class="text-3xl font-bold text-indigo-400 mb-4">PiVision Live Stream</h1>
        <div class="relative bg-gray-800 rounded-xl overflow-hidden shadow-2xl">
            <div class="aspect-w-16 aspect-h-9">
                <img id="video-feed" src="{{ url_for('video_feed') }}" 
                     class="w-full h-auto object-cover" 
                     alt="Live camera feed">
            </div>
            <div class="absolute bottom-4 left-4 bg-black bg-opacity-50 text-white px-3 py-1 rounded-lg text-sm">
                <span id="resolution-display">640×480</span>
            </div>
        </div>
        <div class="mt-6 grid grid-cols-1 md:grid-cols-3 gap-4">
            <div class="bg-gray-800 p-4 rounded-lg">
                <h3 class="text-lg font-semibold text-indigo-300 mb-2">Stream Info</h3>
                <div class="space-y-2">
                    <div class="flex justify-between">
                        <span class="text-gray-400">Status:</span>
                        <span class="text-green-400 font-medium">Active</span>
                    </div>
                    <div class="flex justify-between">
                        <span class="text-gray-400">FPS:</span>
                        <span class="text-white">30</span>
                    </div>
                    <div class="flex justify-between">
                        <span class="text-gray-400">Resolution:</span>
                        <span class="text-white">640×480</span>
                    </div>
                </div>
            </div>
            <div class="bg-gray-800 p-4 rounded-lg">
                <h3 class="text-lg font-semibold text-indigo-300 mb-2">Controls</h3>
                <div class="space-y-3">
                    <div>
                        <label class="block text-gray-400 text-sm mb-1">Brightness</label>
                        <input type="range" min="0" max="100" value="50" class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer">
                    </div>
                    <div>
                        <label class="block text-gray-400 text-sm mb-1">Contrast</label>
                        <input type="range" min="0" max="100" value="50" class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer">
                    </div>
                </div>
            </div>
            <div class="bg-gray-800 p-4 rounded-lg">
                <h3 class="text-lg font-semibold text-indigo-300 mb-2">Recent Snapshots</h3>
                <div class="grid grid-cols-3 gap-2" id="snapshot-gallery">
                    <div class="bg-gray-700 aspect-square rounded flex items-center justify-center text-gray-500">
                        <i data-feather="image"></i>
                    </div>
                    <div class="bg-gray-700 aspect-square rounded flex items-center justify-center text-gray-500">
                        <i data-feather="image"></i>
                    </div>
                    <div class="bg-gray-700 aspect-square rounded flex items-center justify-center text-gray-500">
                        <i data-feather="image"></i>
                    </div>
                </div>
            </div>
        </div>
    </div>
</main>
<script>
feather.replace();
</script>
</body>
</html>
"""

# -----------------------------
# Flask Routes
# -----------------------------
@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# -----------------------------
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
