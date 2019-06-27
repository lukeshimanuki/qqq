from keras.layers import *
from keras.layers.merge import Concatenate
from keras.models import Sequential, Model
from keras.optimizers import *
from keras import initializers
from keras.callbacks import *


def slice_x(x):
    return x[:, 0:1]


def slice_y(x):
    return x[:, 1:2]


def slice_th(x):
    return x[:, 2:3]


class InPathNet:
    def __init__(self, path_net_config):

        self.n_hidden = path_net_config.n_hidden
        self.dense_num = path_net_config.dense_num
        if path_net_config.use_dense:
            model = 'dense'
        else:
            model = 'separate'

        sess = tf.Session()

        if path_net_config.optimizer == 'adam':
            self.opt = Adam(lr=path_net_config.lr, beta_1=0.5)
        else:
            self.opt = Adadelta(lr=path_net_config.lr)

        self.initializer = initializers.glorot_normal()
        self.sess = sess
        self.setup_inputs()
        self.weight_name = self.get_weight_file_name(path_net_config) + '.hdf5'

        if model == 'dense':
            self.clf = self.create_dense_clf()
        else:
            self.clf = self.create_clf()

        print "*******Creating a clf "+self.weight_name+"*******"

    def get_weight_file_name(self, path_net_config):
        fname = ''
        config_variables = vars(path_net_config)
        for var_name, value in zip(config_variables.keys(), config_variables.values()):
            fname += var_name + '_' + str(value) + '_'
        return fname

    def setup_inputs(self):
        self.dim_cg = 3
        self.cg_input = Input(shape=(self.dim_cg,), name='cg', dtype='float32')
        self.dim_ck = 3
        self.ck_input = Input(shape=(self.dim_ck,), name='ck', dtype='float32')
        self.dim_cols = 2
        self.cols_input = Input(shape=(self.dim_cols,), name='cols', dtype='float32')

    def create_dense_clf(self):
        init_ = self.initializer
        dense_num = self.dense_num

        CG_CK = Concatenate(axis=-1)([self.cg_input, self.ck_input])
        H = Dense(dense_num, activation='relu')(CG_CK)
        for i in range(self.n_hidden):
            H = Dense(dense_num, activation='relu')(H)
        disc_output = Dense(1, activation='sigmoid', init=init_)(H)
        disc = Model(input=[self.cg_input, self.ck_input],
                     output=disc_output,
                     name='disc_output')
        disc.compile(loss='binary_crossentropy', optimizer=self.opt, metrics=['accuracy'])
        return disc

    def create_clf(self):
        dense_num = self.dense_num

        x_g = Lambda(slice_x)(self.cg_input)
        y_g = Lambda(slice_y)(self.cg_input)
        th_g = Lambda(slice_th)(self.cg_input)
        x_k = Lambda(slice_x)(self.ck_input)
        y_k = Lambda(slice_y)(self.ck_input)
        th_k = Lambda(slice_th)(self.ck_input)

        Xs = Concatenate(axis=-1)([x_g, x_k])
        Ys = Concatenate(axis=-1)([y_g, y_k])
        Ths = Concatenate(axis=-1)([th_g, th_k])

        H_Xs = Dense(dense_num, activation='relu')(Xs)
        H_Xs = Dense(8, activation='relu')(H_Xs)

        H_Ys = Dense(dense_num, activation='relu')(Ys)
        H_Ys = Dense(8, activation='relu')(H_Ys)

        H_Ths = Dense(dense_num, activation='relu')(Ths)
        H_Ths = Dense(8, activation='relu')(H_Ths)

        H = Concatenate(axis=-1)([H_Xs, H_Ys, H_Ths])
        for i in range(self.n_hidden):
            H = Dense(dense_num, activation='relu')(H)

        disc_output = Dense(1, activation='sigmoid')(H)
        disc = Model(input=[self.cg_input, self.ck_input],
                     output=disc_output,
                     name='disc_output')
        disc.compile(loss='binary_crossentropy', optimizer=self.opt, metrics=['accuracy'])
        return disc

    def train(self, cgs, cks, labels):
        n_data = len(cgs)

        batch_size = np.min([32, int(n_data*0.1)])
        if batch_size == 0:
            print 'batch size too small, n_data is', n_data
            return

        print '****'+self.weight_name+'****'
        checkpointer = ModelCheckpoint(filepath='./cspace_predicates/in_path_net_weights/' + self.weight_name,
                                       verbose=False, save_best_only=True,
                                       save_weights_only=True)
        stopper = EarlyStopping(monitor='val_loss', patience=10, verbose=0)
        self.clf.fit([cgs, cks], labels, validation_split=0.1, callbacks=[checkpointer, stopper],
                     verbose=True,
                     batch_size=32,
                     epochs=500)

    def load_trained_weight(self, fname=None):
        if fname is None:
            print "Loading weight file", self.weight_name
            self.clf.load_weights(self.weight_name)
        else:
            print "Loading weight file", fname
            self.clf.load_weights('./cspace_predicates/in_path_net_weights/' + fname)

