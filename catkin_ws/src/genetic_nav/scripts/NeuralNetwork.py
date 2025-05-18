import numpy as np
from tensorflow.keras import Input, Model
from tensorflow.keras.layers import Dense

def reconstruir_listas(pesos_flat, bias_flat):
    """Convierte dos listas planas en las listas que Keras espera."""
    import numpy as np

    pesos_flat = np.asarray(pesos_flat, dtype=np.float32)
    bias_flat  = np.asarray(bias_flat,  dtype=np.float32)

    # ----- pesos -----
    w0 = pesos_flat[0:80   ].reshape(5, 16)
    w1 = pesos_flat[80:208].reshape(16, 8)
    w2 = pesos_flat[208:232].reshape(8, 3)

    # ----- bias  -----
    b0 = bias_flat[0:16]
    b1 = bias_flat[16:24]
    b2 = bias_flat[24:27]

    return [w0, w1, w2], [b0, b1, b2]


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
