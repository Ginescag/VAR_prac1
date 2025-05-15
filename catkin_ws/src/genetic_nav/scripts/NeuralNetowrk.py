import numpy as np
from tensorflow.keras import Input, Model
from tensorflow.keras.layers import Dense

def build_model_estandar(weights_list, biases_list):
    """
    Crea una red neuronal con arquitectura fija:
        Input(5) → Dense(16, ReLU) → Dense(8, ReLU) → Dense(3, Softmax)

    Parámetros:
    - weights_list: lista de 3 arrays NumPy con los pesos (shape: [(5,16), (16,8), (8,3)])
    - biases_list:  lista de 3 arrays NumPy con los bias   (shape: [(16,), (8,), (3,)])

    Retorna:
    - modelo Keras listo para usar
    """
    x_in = Input(shape=(5,), name='input_layer')
    
    x = Dense(16, activation='relu', name='dense_0')(x_in)
    x = Dense(8, activation='relu', name='dense_1')(x)
    x = Dense(3, activation='softmax', name='dense_2')(x)

    model = Model(inputs=x_in, outputs=x, name='modelo_estandar')

    # Asignar pesos y sesgos manualmente
    for i, layer in enumerate(model.layers[1:]):  # Omitimos input_layer
        layer.set_weights([weights_list[i], biases_list[i]])

    return model
