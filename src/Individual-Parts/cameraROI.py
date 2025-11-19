from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import cv2
import time
import json

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
# Steering State (Gyroscope-based)
# -----------------------------
class SteeringController:
    def __init__(self):
        self.target_angle = 0.0  # Current target heading (accumulated)
        self.max_rate = 1.0      # Maximum degrees per frame to change
        self.deadzone = 1000     # Ignore small differences (noise reduction)
        self.k_integral = 0.01   # How fast to accumulate angle (adjust this!)
        
    def update(self, left_pixels, right_pixels):
        """
        Update target angle based on wall pixel difference
        More left pixels = turn right (increase angle)
        More right pixels = turn left (decrease angle)
        """
        difference = left_pixels - right_pixels
        
        # Apply deadzone to ignore small differences
        if abs(difference) < self.deadzone:
            difference = 0
        
        # Calculate rate of change (angular velocity)
        rate = difference * self.k_integral
        
        # Clamp rate to prevent sudden jumps
        rate = max(-self.max_rate, min(self.max_rate, rate))
        
        # Accumulate the angle (integral control)
        self.target_angle += rate
        
        # Optional: clamp total angle to prevent spinning indefinitely
        # Uncomment if you want limits:
        # self.target_angle = max(-360, min(360, self.target_angle))
        
        return self.target_angle, rate, difference
    
    def reset(self):
        """Reset to initial heading"""
        self.target_angle = 0.0
    
    def set_angle(self, angle):
        """Manually set target angle"""
        self.target_angle = angle

# Global steering controller
steering = SteeringController()


def load_roi_config(path="roi_config.json"):
    try:
        with open(path, "r") as f:
            cfg = json.load(f)
        print("✓ Loaded ROI config")
        return cfg["zones"]
    except:
        print("⚠ Using default ROI config")
        return {
            "Corner_Top": 20,
            "Corner_Bottom": 40,
            "ignore_Top": 20,
            "Corner_LM": 0,
            "Corner_RM": 100,
            "Wall_Top": 30,
            "Wall_Bottom": 70,
            "Right_Wall_RM": 100,
            "Right_Wall_LM": 70,
            "Left_Wall_RM": 30,
            "Left_Wall_LM": 0,
        }

# Color detection for walls
COLOR_LOW  = (0, 0, 0)      # lower HSV bound (black walls)
COLOR_HIGH = (180, 255, 60)  # upper HSV bound


# -----------------------------
# Video Stream Generator
# -----------------------------
def gen_frames():
    zones = load_roi_config()

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h, w, _ = frame.shape

        # ---- ROI pixel boundaries ----
        Ct = int(h * zones["Corner_Top"] / 100)
        Cb = int(h * zones["Corner_Bottom"] / 100)
        Wt = int(h * zones["Wall_Top"] / 100)
        Wb = int(h * zones["Wall_Bottom"] / 100)

        Clm = int(w * zones["Corner_LM"] / 100)
        Crm = int(w * zones["Corner_RM"] / 100)
        Rlm = int(w * zones["Right_Wall_LM"] / 100)
        Rrm = int(w * zones["Right_Wall_RM"] / 100)
        Llm = int(w * zones["Left_Wall_LM"] / 100)
        Lrm = int(w * zones["Left_Wall_RM"] / 100)

        # ------------------- Pixel Counting ---------------------
        mask = cv2.inRange(hsv, COLOR_LOW, COLOR_HIGH)

        # Extract ROIs
        corner_roi = mask[Ct:Cb, Clm:Crm]
        left_roi   = mask[Wt:Wb, Llm:Lrm]
        right_roi  = mask[Wt:Wb, Rlm:Rrm]

        # Count pixels
        corner_pixels = cv2.countNonZero(corner_roi)
        left_pixels   = cv2.countNonZero(left_roi)
        right_pixels  = cv2.countNonZero(right_roi)

        # ------------------- GYROSCOPE STEERING -------------------
        # Update steering based on wall difference
        target_angle, rate, difference = steering.update(left_pixels, right_pixels)

        # ---------------------------------------------------------

        # ---------------- DRAW ROI boxes ------------------------
        cv2.rectangle(frame, (Clm, Ct), (Crm, Cb), (0, 255, 0), 2)      # Corner
        cv2.rectangle(frame, (Llm, Wt), (Lrm, Wb), (255, 0, 0), 2)      # Left
        cv2.rectangle(frame, (Rlm, Wt), (Rrm, Wb), (0, 0, 255), 2)      # Right

        # ---- Draw pixel count text on each ROI ----
        cv2.putText(frame, f"C:{corner_pixels}",
                    (Clm, Ct - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,255,0), 2)

        cv2.putText(frame, f"L:{left_pixels}",
                    (Llm, Wt - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255,0,0), 2)

        cv2.putText(frame, f"R:{right_pixels}",
                    (Rlm, Wt - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,0,255), 2)

        # ------------------- Display Steering Info -------------------
        # Show target angle, rate, and difference
        info_y = h - 80
        
        cv2.putText(frame, f"Target Heading: {target_angle:.1f}°",
                    (10, info_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        
        cv2.putText(frame, f"Rate: {rate:.2f}°/frame",
                    (10, info_y + 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        
        cv2.putText(frame, f"Diff: {difference}",
                    (10, info_y + 55), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        
        # Visual indicator of steering direction
        center_x = w // 2
        indicator_y = h - 40
        
        # Draw a rotating arrow showing target heading
        arrow_length = 60
        arrow_angle_rad = target_angle * 3.14159 / 180  # Convert to radians (removed minus sign)
        end_x = int(center_x + arrow_length * np.sin(arrow_angle_rad))
        end_y = int(indicator_y - arrow_length * np.cos(arrow_angle_rad))
        
        cv2.circle(frame, (center_x, indicator_y), 65, (100, 100, 100), 2)
        cv2.arrowedLine(frame, (center_x, indicator_y), (end_x, end_y), 
                       (0, 255, 255), 4, tipLength=0.3)
        
        # Draw center reference line (0°)
        cv2.line(frame, (center_x, indicator_y - 70), 
                (center_x, indicator_y - 55), (0, 255, 0), 2)

        # Encode frame to JPEG
        ret, buffer = cv2.imencode(".jpg", frame)
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


# -----------------------------
# Enhanced Frontend with Controls
# -----------------------------
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PiVision - Gyro Steering</title>
<script src="https://cdn.tailwindcss.com"></script>
<script src="https://unpkg.com/feather-icons"></script>
</head>
<body class="bg-gray-900 text-gray-100 min-h-screen">
<main class="container mx-auto px-4 py-8">
    <div class="max-w-5xl mx-auto">
        <h1 class="text-3xl font-bold text-indigo-400 mb-4">PiVision - Gyroscope Steering</h1>
        
        <div class="grid grid-cols-1 lg:grid-cols-3 gap-4 mb-6">
            <!-- Main Video Feed -->
            <div class="lg:col-span-2">
                <div class="relative bg-gray-800 rounded-xl overflow-hidden shadow-2xl">
                    <img src="{{ url_for('video_feed') }}" 
                         class="w-full h-auto object-cover" 
                         alt="Live camera feed">
                    <div class="absolute top-4 left-4 bg-black bg-opacity-70 text-white px-3 py-2 rounded-lg">
                        <div class="text-xs text-gray-300">Steering Mode</div>
                        <div class="text-lg font-bold text-green-400">Gyro Integral</div>
                    </div>
                </div>
            </div>
            
            <!-- Control Panel -->
            <div class="space-y-4">
                <div class="bg-gray-800 p-4 rounded-lg">
                    <h3 class="text-lg font-semibold text-indigo-300 mb-3">Steering Parameters</h3>
                    <div class="space-y-3 text-sm">
                        <div>
                            <div class="flex justify-between mb-1">
                                <span class="text-gray-400">Integral Gain (k)</span>
                                <span class="text-white" id="k-value">0.01</span>
                            </div>
                            <input type="range" id="k-slider" min="1" max="50" value="10" 
                                   class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
                                   onchange="updateK(this.value)">
                            <div class="text-xs text-gray-500 mt-1">Controls accumulation speed</div>
                        </div>
                        
                        <div>
                            <div class="flex justify-between mb-1">
                                <span class="text-gray-400">Max Rate (°/frame)</span>
                                <span class="text-white" id="rate-value">5.0</span>
                            </div>
                            <input type="range" id="rate-slider" min="1" max="20" value="50" 
                                   class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
                                   onchange="updateRate(this.value)">
                            <div class="text-xs text-gray-500 mt-1">Maximum turn rate</div>
                        </div>
                        
                        <div>
                            <div class="flex justify-between mb-1">
                                <span class="text-gray-400">Deadzone (pixels)</span>
                                <span class="text-white" id="dead-value">1000</span>
                            </div>
                            <input type="range" id="dead-slider" min="100" max="5000" value="1000" 
                                   class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
                                   onchange="updateDead(this.value)">
                            <div class="text-xs text-gray-500 mt-1">Ignore small differences</div>
                        </div>
                    </div>
                </div>
                
                <div class="bg-gray-800 p-4 rounded-lg">
                    <h3 class="text-lg font-semibold text-indigo-300 mb-3">Actions</h3>
                    <div class="space-y-2">
                        <button onclick="resetSteering()" 
                                class="w-full bg-yellow-600 hover:bg-yellow-700 text-white font-semibold py-2 px-4 rounded transition">
                            <i data-feather="rotate-ccw" class="inline w-4 h-4 mr-2"></i>
                            Reset Heading to 0°
                        </button>
                        <button onclick="setHeading(90)" 
                                class="w-full bg-blue-600 hover:bg-blue-700 text-white font-semibold py-2 px-4 rounded transition">
                            Set Heading to 90°
                        </button>
                        <button onclick="setHeading(-90)" 
                                class="w-full bg-purple-600 hover:bg-purple-700 text-white font-semibold py-2 px-4 rounded transition">
                            Set Heading to -90°
                        </button>
                    </div>
                </div>
                
                <div class="bg-gray-800 p-4 rounded-lg">
                    <h3 class="text-lg font-semibold text-indigo-300 mb-2">How It Works</h3>
                    <div class="text-xs text-gray-400 space-y-2">
                        <p>• <strong>Integral Control:</strong> Angle accumulates over time</p>
                        <p>• <strong>More left pixels:</strong> Target angle increases (turn right)</p>
                        <p>• <strong>More right pixels:</strong> Target angle decreases (turn left)</p>
                        <p>• <strong>Gyroscope:</strong> Follow the target heading</p>
                    </div>
                </div>
            </div>
        </div>
        
        <div class="grid grid-cols-1 md:grid-cols-4 gap-4">
            <div class="bg-gray-800 p-4 rounded-lg">
                <h3 class="text-lg font-semibold text-indigo-300 mb-2">Stream Info</h3>
                <div class="space-y-2 text-sm">
                    <div class="flex justify-between">
                        <span class="text-gray-400">Status:</span>
                        <span class="text-green-400 font-medium">Active</span>
                    </div>
                    <div class="flex justify-between">
                        <span class="text-gray-400">Resolution:</span>
                        <span class="text-white">640×480</span>
                    </div>
                </div>
            </div>
            
            <div class="bg-gray-800 p-4 rounded-lg">
                <h3 class="text-lg font-semibold text-green-300 mb-2">Corner ROI</h3>
                <div class="text-sm text-gray-400">
                    <p>Detects corner markers</p>
                    <div class="w-full h-2 bg-green-600 rounded mt-2"></div>
                </div>
            </div>
            
            <div class="bg-gray-800 p-4 rounded-lg">
                <h3 class="text-lg font-semibold text-blue-300 mb-2">Left Wall ROI</h3>
                <div class="text-sm text-gray-400">
                    <p>Monitors left boundary</p>
                    <div class="w-full h-2 bg-blue-600 rounded mt-2"></div>
                </div>
            </div>
            
            <div class="bg-gray-800 p-4 rounded-lg">
                <h3 class="text-lg font-semibold text-red-300 mb-2">Right Wall ROI</h3>
                <div class="text-sm text-gray-400">
                    <p>Monitors right boundary</p>
                    <div class="w-full h-2 bg-red-600 rounded mt-2"></div>
                </div>
            </div>
        </div>
    </div>
</main>

<script>
feather.replace();

function updateK(value) {
    const k = value / 1000;
    document.getElementById('k-value').textContent = k.toFixed(3);
    // TODO: Send to backend to update steering.k_integral
}

function updateRate(value) {
    const rate = value / 10;
    document.getElementById('rate-value').textContent = rate.toFixed(1);
    // TODO: Send to backend to update steering.max_rate
}

function updateDead(value) {
    document.getElementById('dead-value').textContent = value;
    // TODO: Send to backend to update steering.deadzone
}

function resetSteering() {
    fetch('/reset_steering', {method: 'POST'})
        .then(response => response.json())
        .then(data => console.log('Steering reset:', data));
}

function setHeading(angle) {
    fetch('/set_heading', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({angle: angle})
    })
    .then(response => response.json())
    .then(data => console.log('Heading set:', data));
}
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

@app.route('/reset_steering', methods=['POST'])
def reset_steering():
    steering.reset()
    return {'status': 'ok', 'angle': steering.target_angle}

@app.route('/set_heading', methods=['POST'])
def set_heading():
    from flask import request
    data = request.get_json()
    angle = data.get('angle', 0)
    steering.set_angle(angle)
    return {'status': 'ok', 'angle': steering.target_angle}

# -----------------------------
if __name__ == "__main__":
    import numpy as np  # Needed for arrow calculation
    print("=" * 60)
    print("Gyroscope-Based Steering Control")
    print("=" * 60)
    print("\nParameters:")
    print(f"  • Integral gain (k): {steering.k_integral}")
    print(f"  • Max rate: {steering.max_rate}°/frame")
    print(f"  • Deadzone: {steering.deadzone} pixels")
    print("\nStarting server on http://0.0.0.0:5000")
    print("=" * 60)
    app.run(host='0.0.0.0', port=5000)
