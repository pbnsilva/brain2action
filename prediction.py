from data import MotionDataSource


class MotionPredictor(MotionDataSource):

    def __init__(self, brain_data_source):
        self._brain_data_source = brain_data_source


class CNNMotionPrediction(MotionPredictor):

    def __init__(self, brain_data_source):
        super(CNNMotionPrediction, self).__init__(brain_data_source)


class RNNMotionPrediction(MotionPredictor):

    def __init__(self, brain_data_source):
        super(CNNMotionPrediction, self).__init__(brain_data_source)


class LSTMMotionPrediction(MotionPredictor):

    def __init__(self, brain_data_source):
        super(CNNMotionPrediction, self).__init__(brain_data_source)


class NEATMotionPrediction(MotionPredictor):

    def __init__(self, brain_data_source):
        super(CNNMotionPrediction, self).__init__(brain_data_source)
