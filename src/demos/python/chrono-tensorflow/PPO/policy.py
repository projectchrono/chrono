"""
NN Policy: PPO with KL Penalty (Actor)
Can save model and use it to resume training
Dinamically allocates GPU memory to avoid 
memory saturation during parallel training
(otherwise tensorflow allocates all GPU memory)
"""
import numpy as np
import tensorflow as tf
import os.path


class Policy(object):
    
    def __init__(self, obs_dim, act_dim, kl_targ, env_name, parallel_GPU=False):
        """
        Args:
            obs_dim: num observation dimensions (int)
            act_dim: num action dimensions (int)
            kl_targ: target KL divergence between pi_old and pi_new
            parallel_GPU: if True enables dynamic allocation of GPU memory 
        """
        self.env_name = env_name
        self.multiGPU = parallel_GPU
        self.beta = 1.0  # dynamically adjusted D_KL loss multiplier
        self.eta = 50  # multiplier for D_KL-kl_targ hinge-squared loss
        self.kl_targ = kl_targ
        self.epochs = 20
        self.lr = None           # lr = learning rate
        self.lr_multiplier = 1.0  # dynamically adjust lr when D_KL out of control
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.savedmodel = os.path.isfile("./savedmodel/"+self.env_name+"/Policy/checkpoint") 
        directory = "./savedmodel/"+self.env_name+"/Policy/"
        if self.savedmodel :          #if there's a file with a saved model uses it to restore the NN
               self._restore_model()
        else:                         #else it creates a new one   
               if not os.path.exists(directory):
                      os.makedirs(directory)
               self._build_graph()
               self._init_session()

    def _build_graph(self):
        """ Build and initialize TensorFlow graph """
        self.g = tf.Graph()
        with self.g.as_default():
             self._placeholders()
             self._policy_nn()
             self._logprob()
             self._kl_entropy()
             self._sample()
             self._loss_train_op()
             self.init = tf.global_variables_initializer()
             self.saver = tf.train.Saver()

    def _placeholders(self):
        """ Input placeholders"""
        # observations, actions and advantages:
        self.obs_ph = tf.placeholder(tf.float32, (None, self.obs_dim), 'obs')
        self.act_ph = tf.placeholder(tf.float32, (None, self.act_dim), 'act')
        self.advantages_ph = tf.placeholder(tf.float32, (None,), 'advantages')
        # strength of D_KL loss terms:
        self.beta_ph = tf.placeholder(tf.float32, (), 'beta')
        self.eta_ph = tf.placeholder(tf.float32, (), 'eta')
        # learning rate:
        self.lr_ph = tf.placeholder(tf.float32, (), 'lr')
        # log_vars and means with pi_old (previous step's policy parameters):
        self.old_log_vars_ph = tf.placeholder(tf.float32, (self.act_dim,), 'old_log_vars')
        self.old_means_ph = tf.placeholder(tf.float32, (None, self.act_dim), 'old_means')

    def _policy_nn(self):
        """ Neural net for policy approximation function

        Policy parameterized by Gaussian means and variances. NN outputs mean
         action based on observation. Trainable variables hold log-variances
         for each action dimension (i.e. variances not determined by NN).
        """
        # hidden layer sizes determined by obs_dim and act_dim (hid2 is geometric mean)
        hid1_size = self.obs_dim * 10  # 10 empirically determined
        hid3_size = self.act_dim * 10  # 10 empirically determined
        hid2_size = int(np.sqrt(hid1_size * hid3_size))
        # heuristic to set learning rate based on NN size (tuned on 'Hopper-v1')
        self.lr = 9e-4 / np.sqrt(hid2_size)  # 9e-4 empirically determined
        
        # 3 hidden layers with tanh activations
        out1 = tf.layers.dense(self.obs_ph, hid1_size, tf.tanh,
                              kernel_initializer=tf.random_normal_initializer(
                                  stddev=np.sqrt(1 / self.obs_dim)), name="h1")
        out2 = tf.layers.dense(out1, hid2_size, tf.tanh,
                              kernel_initializer=tf.random_normal_initializer(
                                  stddev=np.sqrt(1 / hid1_size)), name="h2")
        out3 = tf.layers.dense(out2, hid3_size, tf.tanh,
                              kernel_initializer=tf.random_normal_initializer(
                                  stddev=np.sqrt(1 / hid2_size)), name="h3")
        self.means = tf.layers.dense(out3
                                     , self.act_dim,
                                     kernel_initializer=tf.random_normal_initializer(
                                         stddev=np.sqrt(1 / hid3_size)), name="means")
        # logvar_speed is used to 'fool' gradient descent into making faster updates
        # to log-variances. heuristic sets logvar_speed based on network size.
        logvar_speed = (10 * hid3_size) // 48
        log_vars = tf.get_variable('logvars', (logvar_speed, self.act_dim), tf.float32,
                                   tf.constant_initializer(0.0))
        self.log_vars = tf.reduce_sum(log_vars, axis=0) - 1.0
        self.log_vars = tf.identity(self.log_vars, name="log_vars")

        print('Policy Params -- h1: {}, h2: {}, h3: {}, lr: {:.3g}, logvar_speed: {}'
              .format(hid1_size, hid2_size, hid3_size, self.lr, logvar_speed))

    def _logprob(self):
        """ Calculate log probabilities of a batch of observations & actions

        Calculates log probabilities using previous step's model parameters and
        new parameters being trained.
        """
        # Consider the log probability of a gaussian distribution:
        # https://math.stackexchange.com/questions/892832/why-we-consider-log-likelihood-instead-of-likelihood-in-gaussian-distribution
        logp = -0.5 * tf.reduce_sum(self.log_vars)
        logp += -0.5 * tf.reduce_sum(tf.square(self.act_ph - self.means) /
                                     tf.exp(self.log_vars), axis=1)
        self.logp = logp
        self.logp = tf.identity(self.logp, name="logp")

        logp_old = -0.5 * tf.reduce_sum(self.old_log_vars_ph)
        logp_old += -0.5 * tf.reduce_sum(tf.square(self.act_ph - self.old_means_ph) /
                                         tf.exp(self.old_log_vars_ph), axis=1)
        self.logp_old = logp_old
        self.logp_old = tf.identity(self.logp_old, name="logp_old")

    def _kl_entropy(self):
        """
        Add to Graph:
            1. KL divergence between old and new distributions
            2. Entropy of present policy given states and actions
        """
        log_det_cov_old = tf.reduce_sum(self.old_log_vars_ph)
        log_det_cov_new = tf.reduce_sum(self.log_vars)
        tr_old_new = tf.reduce_sum(tf.exp(self.old_log_vars_ph - self.log_vars))
        #KL Divergence formultivariate normal ditributions
        #https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence#Multivariate_normal_distributions
        # log(sigma1/sigma0) = log(sigma1)-log(sigma0) 
        # tr: matrix trace
        self.kl = 0.5 * tf.reduce_mean(log_det_cov_new - log_det_cov_old + tr_old_new +
                                       # (mu1-mu0 )T*SIGMA^-1*(mu1-mu0):
                                       tf.reduce_sum(tf.square(self.means - self.old_means_ph) /
                                                     tf.exp(self.log_vars), axis=1) -
                                       self.act_dim)
                                       # k  = act_dim;
        self.kl = tf.identity(self.kl, name="kl")
                                       
        # simply the entropy formula of a multivariate normal distribution
        # https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Entropy
        self.entropy = 0.5 * (self.act_dim * (np.log(2 * np.pi) + 1) +
                              tf.reduce_sum(self.log_vars))
        self.entropy = tf.identity(self.entropy, name="entropy")

    def _sample(self):
        """ Sample from distribution, given observation """
        self.sampled_act = tf.add(self.means ,
                            #TODO: veriance is artificially reduced: consider eliminating the division by 2
                            # Less variance means actions are closer to the mean (reducing exploration but "standardizing" the behavior)
                            tf.exp(self.log_vars / 2.0) *tf.random_normal(shape=(self.act_dim,)),
                             name = 'sampledact')


    def _loss_train_op(self):
        """
        Three loss terms:
            1) standard policy gradient
            2) D_KL(pi_old || pi_new)
            3) Hinge loss on [D_KL - kl_targ]^2

        See: https://arxiv.org/pdf/1707.02286.pdf
        """
        loss1 = -tf.reduce_mean(self.advantages_ph *                   # loss1 = E [(PI/PIold)*A]
                                tf.exp(self.logp - self.logp_old))     # PI/PIold = e^(logprob) / e^(logprob_old) = e^(logprob-logprob_old)
        loss2 = tf.reduce_mean(self.beta_ph * self.kl)

        loss3 = self.eta_ph * tf.square(tf.maximum(0.0, self.kl - 2.0 * self.kl_targ))
        self.loss = loss1 + loss2 + loss3
        self.loss = tf.identity(self.loss, name="loss")
        optimizer = tf.train.AdamOptimizer(self.lr_ph)
        self.train_op = optimizer.minimize(self.loss, name='train_op')


    def _init_session(self):
        """Launch TensorFlow session and initialize variables
        if multiprocessing on gpu  enable dynamic memory allocation"""
        if self.multiGPU :
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            self.sess = tf.Session(graph=self.g, config=config)
        else:
            self.sess = tf.Session(graph=self.g)
        self.sess.run(self.init)
        
    def _restore_model(self):
           """ restore saved model. 
           if multiprocessing on gpu  enable dynamic memory allocation """
           tf.reset_default_graph()
           if self.multiGPU :
                config = tf.ConfigProto()
                config.gpu_options.allow_growth = True
                self.sess = tf.Session(config=config)
           else:

               self.sess = tf.Session()

           # import graph from file, initialize variables, get variables and operations needed, initialize saver, restore checkpoint
           loader = tf.train.import_meta_graph("./savedmodel/"+self.env_name+"/Policy/trained_variables.ckpt.meta")
           self.sess.run(tf.global_variables_initializer())           
           self.g = tf.get_default_graph()
           self.obs_ph = self.g.get_tensor_by_name('obs:0') 
           self.act_ph = self.g.get_tensor_by_name('act:0')
           self.means  = self.g.get_tensor_by_name('means/BiasAdd:0')
           self.log_vars  = self.g.get_tensor_by_name('log_vars:0')
           self.advantages_ph  = self.g.get_tensor_by_name('advantages:0')
           self.beta_ph  = self.g.get_tensor_by_name('beta:0')
           self.eta_ph  = self.g.get_tensor_by_name('eta:0')
           self.lr_ph  = self.g.get_tensor_by_name('lr:0')
           self.old_log_vars_ph  = self.g.get_tensor_by_name('old_log_vars:0')
           self.old_means_ph  = self.g.get_tensor_by_name('old_means:0')
           self.sampled_act  = self.g.get_tensor_by_name('sampledact:0')
           self.loss  = self.g.get_tensor_by_name('loss:0')
           self.train_op  = self.g.get_operation_by_name('train_op') 
           self.entropy  = self.g.get_tensor_by_name('entropy:0')
           self.kl  = self.g.get_tensor_by_name('kl:0')
           self.logp  = self.g.get_tensor_by_name('logp:0')
           self.logp_old  = self.g.get_tensor_by_name('logp_old:0')
           self.lr = 9e-4 / np.sqrt(int(np.sqrt(self.obs_dim * 10 * self.act_dim * 10)))
           self.saver = tf.train.Saver()
           loader.restore(self.sess, tf.train.latest_checkpoint("./savedmodel/"+self.env_name+"/Policy"))

    def sample(self, obs):
        """Draw sample from policy distribution"""
        feed_dict = {self.obs_ph: obs}

        return self.sess.run(self.sampled_act, feed_dict=feed_dict)

    def update(self, observes, actions, advantages, logger):
        """ Update policy based on observations, actions and advantages

        Args:
            observes: observations, shape = (N, obs_dim)
            actions: actions, shape = (N, act_dim)
            advantages: advantages, shape = (N,)
            logger: Logger object, see utils.py
        """
        feed_dict = {self.obs_ph: observes,
                     self.act_ph: actions,
                     self.advantages_ph: advantages,
                     self.beta_ph: self.beta,
                     self.eta_ph: self.eta,
                     self.lr_ph: self.lr * self.lr_multiplier}
        old_means_np, old_log_vars_np = self.sess.run([self.means, self.log_vars],
                                                      feed_dict)
        feed_dict[self.old_log_vars_ph] = old_log_vars_np
        feed_dict[self.old_means_ph] = old_means_np
        loss, kl, entropy = 0, 0, 0
        for e in range(self.epochs):
            # TODO: need to improve data pipeline - re-feeding data every epoch
            self.sess.run(self.train_op, feed_dict)
            loss, kl, entropy = self.sess.run([self.loss, self.kl, self.entropy], feed_dict)
            if kl > self.kl_targ * 4:  # early stopping if D_KL diverges badly
                break
        # TODO: too many "magic numbers" in next 8 lines of code, need to clean up
        #  BETA and learning rate empirical tuning
        if kl > self.kl_targ * 2:  # servo beta to reach D_KL target
            self.beta = np.minimum(35, 1.5 * self.beta)  # max clip beta
            if self.beta > 30 and self.lr_multiplier > 0.1:
                self.lr_multiplier /= 1.5
        elif kl < self.kl_targ / 2:
            self.beta = np.maximum(1 / 35, self.beta / 1.5)  # min clip beta
            if self.beta < (1 / 30) and self.lr_multiplier < 10:
                self.lr_multiplier *= 1.5
        # save model checkpoint 
        self.saver.save(self.sess, "./savedmodel/"+self.env_name+"/Policy/trained_variables.ckpt")
        logger.log({'PolicyLoss': loss,
                    'PolicyEntropy': entropy,
                    'KL': kl,
                    'Beta': self.beta,
                    '_lr_multiplier': self.lr_multiplier})

    def close_sess(self):
        #self.saver.save(self.sess, "./savedmodel/"+self.env_name+"/Policy/final_variables.ckpt")
        """ Close TensorFlow session """
        self.sess.close()
