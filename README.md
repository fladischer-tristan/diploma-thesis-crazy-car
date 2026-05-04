# Crazy Car - Semi-Autonomous Driving

This project enables a Crazy Car (Arduino V2.0) to learn and replicate driving routes through imitation learning. The user first drives the car manually to collect training data, then trains a model, and finally uploads it to the car for autonomous driving.

---

## What you need

- Crazy Car Controller Arduino V2.0
- Arduino IDE (or PlatformIO)
- Python 3.8+ with tensorflow, numpy, pandas, scikit-learn
- USB cable (data sync)
- MicroSD card (for collecting training data)

---

## How it works

1. Collect training data: Drive the car manually along your desired path. The car logs sensor readings and your steering commands.

2. Train the imitation learning model: Use the Python script to turn the logged data into a neural network model.

3. Port the model to the car: Convert the trained model to a format the Arduino can run (e.g., TensorFlow Lite Micro).

4. Run autonomous mode: Power on the car and press the Start button. The car follows the learned route.

---

## Setup and installation

### On your computer

```bash
# Clone the repo (replace with actual URL)
git clone https://github.com/fladischer-tristan/diploma-thesis-crazy-car.git
cd diploma-thesis-crazy-car

# Install Python dependencies
pip install -r requirements.txt
