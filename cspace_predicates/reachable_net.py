from keras.layers import *
from keras.layers.merge import Concatenate
from keras.models import Sequential, Model
from keras.optimizers import *
from keras import initializers
from keras.callbacks import *

import tensorflow as tf


def slice_rel_cg(x):
    return x[:, -3:]


def slice_key_config_collisions(x):
    return x[:, :-3]


class ReachableNet:
    def __init__(self, parameters, n_key_configs):
        # this is a dense neural network that takes inputs from path detector and generate a classification result
        self.n_neurons_konf = parameters.n_neurons_konf
        self.n_layers_konf = parameters.n_layers_konf
        self.n_neurons_rel_cg = parameters.n_neurons_rel_cg
        self.n_layers_rel_cg = parameters.n_layers_rel_cg

        self.n_key_configs = n_key_configs
        self.lr = parameters.lr

        if parameters.optimizer == 'adam':
            self.opt = Adam(lr=parameters.lr, beta_1=0.5)
        elif parameters.optimizer == 'adadelta':
            self.opt = Adadelta(lr=parameters.lr)
        self.optimizer = parameters.optimizer

        self.x_input = Input(shape=(self.n_key_configs * 2 + 3,), name='kval', dtype='float32')

        self.random_seed = parameters.random_seed
        self.network = self.create_network_with_rel_cg()
        self.weight_name = self.get_weight_file_name()

    def get_weight_file_name(self):
        path = './reachable_clfs/NN_weights/'
        fname = str(self.n_layers_konf) + '_' + str(self.n_neurons_konf) + '_' + str(self.n_layers_rel_cg) \
                + '_' + str(self.n_neurons_rel_cg) + '_' + str(self.lr) + '_' + str(self.random_seed) + '_' + \
                str(self.optimizer) + '.hdf5'
        return path + fname

    def create_network_with_rel_cg(self):
        key_config_collisions = Lambda(slice_key_config_collisions)(self.x_input)
        H = Dense(self.n_neurons_konf, activation='relu')(key_config_collisions)
        for i in range(self.n_layers_konf):
            H = Dense(self.n_neurons_konf, activation='relu')(H)

        rel_cg_input = Lambda(slice_rel_cg)(self.x_input)
        H_rel_cg = Dense(self.n_neurons_rel_cg, activation='relu')(rel_cg_input)
        for i in range(self.n_layers_rel_cg):
            H_rel_cg = Dense(self.n_neurons_rel_cg, activation='relu')(H_rel_cg)

        H = Concatenate(axis=-1)([H, H_rel_cg])
        # I could potentially add more layers here
        disc_output = Dense(1, activation='sigmoid')(H)
        discriminator = Model(input=[self.x_input],
                              output=disc_output,
                              name='disc_output')
        discriminator.compile(loss='binary_crossentropy', optimizer=self.opt)
        return discriminator

    def predict(self, x_values):
        return (self.network.predict(x_values) > 0.5).squeeze()

    def train(self, x_values, y_values):
        n_data = len(x_values)
        batch_size = np.min([32, int(n_data * 0.1)])

        print '****' + self.weight_name + '****'
        checkpointer = ModelCheckpoint(filepath=self.weight_name, verbose=False, save_best_only=True,
                                       save_weights_only=True)

        stopper = EarlyStopping(monitor='val_loss', patience=50, verbose=0)
        self.network.fit(x_values, y_values, validation_split=0.1,
                         callbacks=[checkpointer, stopper], verbose=True,
                         batch_size=batch_size, epochs=1000)

    def load_best_model(self):
        self.network.load_weights(self.weight_name)
