import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split

# ----------------------------
# 1) Load data
# ----------------------------
df = pd.read_csv("home_straight_path_V1.csv", sep=";", encoding="cp1252")
df.columns = df.columns.str.strip()

feature_cols = [
    "packetNumber",
    "leftDistance", "middleDistance", "rightDistance",
    "ax", "ay", "az", "gx", "gy", "gz",
    "velocity"
]

target_cols = [
    "servoPulse",
    "escPulse"
]

all_cols = feature_cols + target_cols
# Convert to numeric; invalid entries (like "Jän.25") become NaN
for c in all_cols:
    df[c] = pd.to_numeric(df[c], errors="coerce")

# Drop rows with any NaN in required columns
before = len(df)
df = df.dropna(subset=all_cols)
after = len(df)
print(f"Dropped {before - after} corrupted rows.")


X_raw = df[feature_cols].to_numpy(dtype=np.float32)
Y_raw = df[target_cols].to_numpy(dtype=np.float32)

""" Only for getting min/max constraints: """
x_min_constraints = X_raw.min(axis=0)
x_max_constraints = X_raw.max(axis=0)

y_min_constraints = Y_raw.min(axis=0)
y_max_constraints = Y_raw.max(axis=0)

print("\n===== FEATURE MIN/MAX CONSTRAINTS =====")
for name, mn, mx in zip(feature_cols, x_min_constraints, x_max_constraints):
    print(f"{name:15s}  MIN={mn:.9f}   MAX={mx:.9f}")

print("\n===== TARGET MIN/MAX CONSTRAINTS =====")
for name, mn, mx in zip(target_cols, y_min_constraints, y_max_constraints):
    print(f"{name:15s}  MIN={mn:.9f}   MAX={mx:.9f}")

# ----------------------------
# 2) Normalize to [-1, 1]
# (Placeholder: fill in real min/max later)
# ----------------------------
# These should be arrays with shape (n_features,) and (n_targets,)
x_min = np.array([0]*len(feature_cols), dtype=np.float32)
x_max = np.array([1]*len(feature_cols), dtype=np.float32)

y_min = np.array([0]*len(target_cols), dtype=np.float32)
y_max = np.array([1]*len(target_cols), dtype=np.float32)

def minmax_to_minus1_plus1(x, xmin, xmax, eps=1e-9):
    """
    Scale value x to range [-1; 1]
    """
    x = (x - xmin) / (xmax - xmin + eps)      # -> [0, 1]
    x = 2.0 * x - 1.0                         # -> [-1, 1]
    return np.clip(x, -1.0, 1.0)

X = minmax_to_minus1_plus1(X_raw, x_min, x_max)
Y = minmax_to_minus1_plus1(Y_raw, y_min, y_max)

# ----------------------------
# 3) Split train/val
# ----------------------------
X_train, X_val, Y_train, Y_val = train_test_split(
    X, Y, test_size=0.2, random_state=42
)

# ----------------------------
# 4) Define small MLP
# ----------------------------
input_dim = X.shape[1]
output_dim = Y.shape[1]

model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(input_dim,)),
    tf.keras.layers.Dense(16, activation="tanh"),
    tf.keras.layers.Dense(16, activation="tanh"),
    tf.keras.layers.Dense(output_dim, activation="tanh"),  # outputs in [-1, 1]
])

model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3),
    loss="mse"
)

history = model.fit(
    X_train, Y_train,
    validation_data=(X_val, Y_val),
    epochs=30,
    batch_size=64,
    verbose=1
)

# ----------------------------
# 5) Export to TFLite (float32)
# ----------------------------
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

with open("imitation_model_float32.tflite", "wb") as f:
    f.write(tflite_model)

print("Saved imitation_model_float32.tflite")


# some evaluation
print("X shape:", X.shape, "Y shape:", Y.shape)

pred = model.predict(X_val[:10])
print("First 3 true Y:", Y_val[:3])
print("First 3 pred Y:", pred[:3])

val_mse = model.evaluate(X_val, Y_val, verbose=0)
print("Validation MSE:", val_mse)


# equivalence test
import numpy as np
import tensorflow as tf

# Load TFLite model
interpreter = tf.lite.Interpreter(model_path="imitation_model_float32.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print("TFLite input:", input_details)
print("TFLite output:", output_details)

# Choose one sample from validation set
x = X_val[0:1].astype(np.float32)  # shape (1, input_dim)

# Run TFLite inference
interpreter.set_tensor(input_details[0]["index"], x)
interpreter.invoke()
tflite_out = interpreter.get_tensor(output_details[0]["index"])

# Run Keras inference
keras_out = model.predict(x, verbose=0)

print("Keras out: ", keras_out)
print("TFLite out:", tflite_out)
print("Abs diff: ", np.abs(keras_out - tflite_out))