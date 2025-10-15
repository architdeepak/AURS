import tensorflow as tf
import numpy as np
import cv2

# Load the saved model
model_path = "YOLOMODEL"  # Update this with your actual model path
model = tf.saved_model.load(model_path)
print("Model loaded successfully!")

# Load and preprocess an image
image_path = "image.png"  # Update with your image path
image = cv2.imread(image_path)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
image_resized = cv2.resize(image, (640, 640))  # Resize to match model input size
image_normalized = image_resized / 255.0  # Normalize pixel values
image_input = np.expand_dims(image_normalized, axis=0)  # Add batch dimension

# Convert to tensor
input_tensor = tf.convert_to_tensor(image_input, dtype=tf.float32)

# Run inference by calling the model directly
try:
    output = model(input_tensor)  # Call model directly
    print("Model inference successful!")
except Exception as e:
    print("Error during inference:", str(e))
    exit()

# Inspect the shape and content of the output tensor
print("Output shape:", output.shape)
print("Output tensor:", output)

# YOLO usually outputs a tensor with shape (1, grid_size, grid_size, num_classes + 5)
# Assuming output is a tensor of shape (1, 84, 8400), we need to reshape or split this

# Post-process the output tensor based on the model's architecture
# For example, we will assume the model outputs detections in a flattened format
output_tensor = output[0]  # Get rid of the batch dimension

# Reshape output to understand the structure
# Assuming 8400 is a flattened array of detections
# You might need to reshape this based on the model's configuration
grid_size = 84  # Check model documentation for grid size
num_classes = 80  # Adjust based on your model's class count
num_boxes = 3  # YOLO typically uses 3 boxes per grid cell (this can vary)

output_reshaped = tf.reshape(output_tensor, (grid_size, grid_size, num_boxes, 5 + num_classes))

# Process each grid cell and extract information
for i in range(grid_size):
    for j in range(grid_size):
        for b in range(num_boxes):
            box = output_reshaped[i, j, b, :4]  # Get the box coordinates (x, y, w, h)
            confidence = output_reshaped[i, j, b, 4]  # Get the confidence score
            class_probs = output_reshaped[i, j, b, 5:]  # Get the class probabilities

            # If the confidence is above a threshold, it's a valid detection
            if confidence > 0.5:
                class_id = np.argmax(class_probs)  # Get the class with the highest probability
                class_score = class_probs[class_id]  # Get the corresponding score

                print(f"Detected object: {class_id} with confidence: {confidence:.2f} and score: {class_score:.2f}")
                # Optionally, map the class_id to a label
                class_labels = {
                    0: "person",
                    1: "bicycle",
                    2: "car",
                    3: "motorcycle",
                    4: "airplane",
                    5: "bus",
                    6: "train",
                    7: "truck",
                    8: "boat",
                    9: "traffic light",
                }
                detected_object = class_labels.get(class_id, f"unknown_{class_id}")
                print(f"Detected object: {detected_object}")
else:
    print("No valid detections found.")
