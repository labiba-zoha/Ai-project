import cv2 as cv
import numpy as np
from tensorflow.keras import datasets, layers, models

# Load CIFAR-10
(train_images, train_labels), (test_images, test_labels) = datasets.cifar10.load_data()

# Normalize pixel values
train_images, test_images = train_images / 255.0, test_images / 255.0

# Correct class names
class_names = ['plane','car','bird','cat','dog','frog','horse','ship','boat','truck']

# Limit dataset size (optional)
train_images = train_images[:20000]
train_labels = train_labels[:20000]
test_images = test_images[:4000]
test_labels = test_labels[:4000]

# Load pre-trained model
model = models.load_model('image_classifier.model')

# Read and preprocess your image
img = cv.imread('cat2.jpg')
img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
img = cv.resize(img, (32, 32))  # Resize to CIFAR-10 size

# Predict
prediction = model.predict(np.array([img]) / 255.0)
index = np.argmax(prediction)
print(f'Prediction is: {class_names[index]}')
