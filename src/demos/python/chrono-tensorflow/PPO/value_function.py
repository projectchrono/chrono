"""
State-Value Function

"""

import tensorflow as tf
import numpy as np
from sklearn.utils import shuffle
import os.path

class NNValueFunction(object):
    """ NN-based state-value function """
    def __init__(self, obs_dim, env_name, MultiGPU = False):
        """
        Args:
            obs_dim: number of dimensions in observation vector (int)
        """
        self.env_name = env_name
        self.multiGPU = MultiGPU
        self.replay_buffer_x = None
        self.replay_buffer_y = None
        self.obs_dim = obs_dim
        self.epochs = 10
        self.lr = None  # learning rate set in _build_graph()
        self.savedmodel = os.path.isfile("./savedmodel/"+self.env_name+"/VF/checkpoint")
        directory = "./savedmodel/"+self.env_name+"/VF/"
        if self.savedmodel :
               self._restore()
        else:
               if not os.path.exists(directory):
                      os.makedirs(directory)
               self._build_graph()
               #self.sess = tf.Session(graph=self.g)
               #self.sess.run(self.init)

    def _build_graph(self):
        """ Construct TensorFlow graph, including loss function, init op and train op """
        self.g = tf.Graph()
        with self.g.as_default():
            self.obs_ph = tf.placeholder(tf.float32, (None, self.obs_dim), 'obs_valfunc')
            self.val_ph = tf.placeholder(tf.float32, (None,), 'val_valfunc')
            # hid1 layer size is 10x obs_dim, hid3 size is 10, and hid2 is geometric mean
            hid1_size = self.obs_dim * 10  # 10 chosen empirically on 'Hopper-v1'
            hid3_size = 5  # 5 chosen empirically on 'Hopper-v1'
            hid2_size = int(np.sqrt(hid1_size * hid3_size))
            # heuristic to set learning rate based on NN size (tuned on 'Hopper-v1')
            self.lr = 1e-2 / np.sqrt(hid2_size)  # 1e-3 empirically determined
            print('Value Params -- h1: {}, h2: {}, h3: {}, lr: {:.3g}'
                  .format(hid1_size, hid2_size, hid3_size, self.lr))
            # 3 hidden layers with tanh activations
            out = tf.layers.dense(self.obs_ph, hid1_size, tf.tanh,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=np.sqrt(1 / self.obs_dim)), name="h1VF")
            out = tf.layers.dense(out, hid2_size, tf.tanh,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=np.sqrt(1 / hid1_size)), name="h2VF")
            out = tf.layers.dense(out, hid3_size, tf.tanh,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=np.sqrt(1 / hid2_size)), name="h3VF")
            out = tf.layers.dense(out, 1,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=np.sqrt(1 / hid3_size)), name='output')
            self.out = tf.squeeze(out)
            self.loss = tf.reduce_mean(tf.square(self.out - self.val_ph), name='lossVF')  # squared loss
            optimizer = tf.train.AdamOptimizer(self.lr)
            self.train_op = optimizer.minimize(self.loss, name='train_opVF')
            self.init = tf.global_variables_initializer()
            self.saverVF = tf.train.Saver()
        if self.multiGPU :
                config = tf.ConfigProto()
                config.gpu_options.allow_growth = True
                #config.gpu_options.per_process_gpu_memory_fraction = 0.1
                self.sess = tf.Session(graph=self.g, config=config)
        else:

               self.sess = tf.Session(graph=self.g)
        self.sess.run(self.init)
        
    def _restore(self):
           if self.multiGPU :
                config = tf.ConfigProto()
                config.gpu_options.allow_growth = True
                #config.gpu_options.per_process_gpu_memory_fraction = 0.1
                self.sess = tf.Session(config=config)
           else:

               self.sess = tf.Session()        
           loader = tf.train.import_meta_graph("./savedmodel/"+self.env_name+"/VF/trained_VF.ckpt.meta")
           self.sess.run(tf.global_variables_initializer())
           self.g = tf.get_default_graph()
           self.obs_ph = self.g.get_tensor_by_name('obs_valfunc:0') 
           self.val_ph = self.g.get_tensor_by_name('val_valfunc:0')
           out  = self.g.get_tensor_by_name('output/BiasAdd:0')
           self.out = tf.squeeze(out)
           self.loss  = self.g.get_tensor_by_name('lossVF:0')
           self.train_op  = self.g.get_operation_by_name('train_opVF') 
           self.lr =  1e-2 / np.sqrt(int(np.sqrt(self.obs_dim * 10 * 5)))
           self.saverVF = tf.train.Saver()
           loader.restore(self.sess, tf.train.latest_checkpoint("./savedmodel/"+self.env_name+"/VF"))

    def fit(self, x, y, logger):
        """ Fit model to current data batch + previous data batch

        Args:
            x: features
            y: target
            logger: logger to save training loss and % explained variance
        """
        
        # minibatches of 256 tuples. Trining set is pretty big when the episode is long (steps_perepisode*episode_per batch)
        num_batches = max(x.shape[0] // 256, 1)
        batch_size = x.shape[0] // num_batches
        y_hat = self.predict(x)  # check explained variance prior to update
        old_exp_var = 1 - np.var(y - y_hat)/np.var(y)
        #alla prima iterazione buffer coincide con ultimo dato, a quelle seguenti lo incollo al buffer
        if self.replay_buffer_x is None:
            x_train, y_train = x, y
        else:
            x_train = np.concatenate([x, self.replay_buffer_x])
            y_train = np.concatenate([y, self.replay_buffer_y])
            # flush buffar. the last one wil be appended to the next
        self.replay_buffer_x = x
        self.replay_buffer_y = y
        for e in range(self.epochs):
            x_train, y_train = shuffle(x_train, y_train)
            for j in range(num_batches):
                start = j * batch_size
                end = (j + 1) * batch_size
                feed_dict = {self.obs_ph: x_train[start:end, :],
                             self.val_ph: y_train[start:end]}
                _, l = self.sess.run([self.train_op, self.loss], feed_dict=feed_dict)
        y_hat = self.predict(x)
        loss = np.mean(np.square(y_hat - y))         # explained variance after update
        exp_var = 1 - np.var(y - y_hat) / np.var(y)  # diagnose over-fitting of val func

        self.saverVF.save(self.sess, "./savedmodel/"+self.env_name+"/VF/trained_VF.ckpt")
        logger.log({'ValFuncLoss': loss,
                    'ExplainedVarNew': exp_var,
                    'ExplainedVarOld': old_exp_var})

    def predict(self, x):
        """ Predict method """
        feed_dict = {self.obs_ph: x}
        y_hat = self.sess.run(self.out, feed_dict=feed_dict)

        return np.squeeze(y_hat)

    def close_sess(self):
        """ Close TensorFlow session """
        self.saverVF.save(self.sess, "./savedmodel/"+self.env_name+"/VF/trained_VF.ckpt")
        self.sess.close()
