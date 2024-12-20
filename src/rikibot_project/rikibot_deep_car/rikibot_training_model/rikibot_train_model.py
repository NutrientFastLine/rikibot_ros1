# -*- coding: utf-8 -*-
import os
import sys
sys.path.append("../parts")
sys.path.append("../util")

from keras import KerasLinear
from datastore import TubGroup, TubWriter


TRAIN_TEST_SPLIT = 0.8
BATCH_SIZE = 48


def train(tub_names, new_model_path):
    """
    use the specified data in tub_names to train an artifical neural network
    saves the output trained model as model_name
    """
    X_keys = ['cam/image_array']
    y_keys = ['user/angle', 'user/throttle']

    new_model_path = os.path.expanduser(new_model_path)

    kl = KerasLinear()

    print('tub_names', tub_names)
    if not tub_names:
        print("train data not exist!")
        #tub_names = os.path.join(cfg.DATA_PATH, '*')
    tubgroup = TubGroup(tub_names)
    train_gen, val_gen = tubgroup.get_train_val_gen(X_keys, y_keys,
                                                    batch_size=BATCH_SIZE,
                                                    train_frac=TRAIN_TEST_SPLIT)

    total_records = len(tubgroup.df)
    total_train = int(total_records * TRAIN_TEST_SPLIT)
    total_val = total_records - total_train
    print('train: %d, validation: %d' % (total_train, total_val))
    steps_per_epoch = total_train // BATCH_SIZE
    print('steps_per_epoch', steps_per_epoch)


    kl.train(train_gen,
             val_gen,
             saved_model_path=new_model_path,
             steps=steps_per_epoch,
             train_split=TRAIN_TEST_SPLIT)

if __name__ == '__main__':
    dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path = dir_path.replace('rikibot_training_model', '')
    train_path = '/home/rikibot/Work/train_data'
    models_path = dir_path + 'rikibot_autopilot/models/rikipolit'
    train(train_path, models_path)





