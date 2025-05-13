import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

# Definir el autoencoder
def create_autoencoder(num_points=1024):
    inputs = keras.Input(shape=(num_points, 3))
    x = layers.Conv1D(64, 1, activation='relu')(inputs)
    x = layers.Conv1D(128, 1, activation='relu')(x)
    x = layers.GlobalMaxPooling1D()(x)
    encoded = layers.Dense(256, activation='relu')(x)

    x = layers.Dense(1024, activation='relu')(encoded)
    x = layers.Reshape((num_points, 1))(x)
    x = layers.Conv1DTranspose(128, 1, activation='relu')(x)
    x = layers.Conv1DTranspose(64, 1, activation='relu')(x)
    decoded = layers.Conv1DTranspose(3, 1)(x)

    autoencoder = keras.Model(inputs, decoded)
    return autoencoder
