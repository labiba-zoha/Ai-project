import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras import datasets, layers, models

# Load CIFAR-10
(train_images, train_labels), (test_images, test_labels) = datasets.cifar10.load_data()

# Normalize pixel values
train_images, test_images = train_images / 255.0, test_images / 255.0

class_names = ['plane','car','bird','cat','Deer','dog','frog','horse','ship','truck']




train_images = train_images[:50000]
train_labels = train_labels[:50000]
test_images = test_images[:10000]
test_labels = test_labels[:10000]

# Define model
model = models.Sequential()
model.add(layers.Conv2D(32, (3,3), activation='relu', input_shape=(32,32,3)))
model.add(layers.MaxPooling2D((2,2)))
model.add(layers.Conv2D(64, (3,3), activation='relu'))
model.add(layers.MaxPooling2D((2,2)))
model.add(layers.Conv2D(64, (3,3), activation='relu'))
model.add(layers.Flatten())
model.add(layers.Dense(64, activation='relu'))
model.add(layers.Dense(10, activation='softmax'))

# Compile model
model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# Train model
model.fit(train_images, train_labels, epochs=10, validation_data=(test_images, test_labels))

# Evaluate model
loss, accuracy = model.evaluate(test_images, test_labels)  
print(f"Loss: {loss}")        
print(f"Accuracy: {accuracy}")  

# Save model
model.save('image_classifier.model')  
