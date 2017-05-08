#!/usr/bin/env python
# coding: utf8

"""
bench to test the take-off and apogee detection algo
"""

import sys
import csv
import math
import random


class Accel(object):
    """extract accelearation fields from successive csv rows"""
    def __init__(self, sampling_rate=0.1, extra_samples=1, noise=0.1, bias=0.1):
        """
        the sampling rate in [s] is used to have a constant time increment
        the extra samples are added to simulate the time before 0 s
        the noise is a factor of error
        the bias in [g] is added to the acceleration
        """
        self._sampling_rate = sampling_rate
        self._extra_samples = extra_samples
        self._noise = noise
        self._bias = bias
        print('noise = %5.2f, bias = %5.2f' % (noise, bias))

        # column indexes
        self.indexes = {
            'temps': None,
            'beta': None,
            'poussée': None,
            'poids': None,
            'rampe': None,
            'trainée': None,
        }

        # column content in [s, m/s^2, m/s^2, m/s^2]
        self.data = {
            'temps': [],
            'beta': [],
            'poussée': [],
            'poids': [],
            'rampe': [],
            'trainée': [],
            'acc long': [],
            'acc lat': [],
        }

        self._computed_data = {
            'temps': [],
            'acc long': [],
            'acc lat': [],
        }
        # takeoff and apogee times extract from StabTraj data in [s]
        self.apogee_time = None

        random.seed(1)

        self._iteration_step = 0

    def _indexes_lookup(self, row):
        """extract indexes from row"""
        for i in range(len(row)):
            row_part = row[i]
            if 't' == row_part:
                self.indexes['temps'] = i
            if 'Beta' == row_part:
                self.indexes['beta'] = i
            if 'Poussée' == row_part:
                self.indexes['poussée'] = i
            if 'Poids' == row_part:
                self.indexes['poids'] = i
            if 'R_rampe' == row_part:
                self.indexes['rampe'] = i
            if 'Trainée' == row_part:
                self.indexes['trainée'] = i

    def process(self, row):
        """process the given row to find indexes or extract content"""
        # check for indexes
        if self.indexes['temps'] is None:
            self._indexes_lookup(row)
            return

        # check for takeoff time
        if 'Sortie de rampe' in row:
            conv = row[self.indexes['temps']].replace(',', '.')
            takeoff_time = float(conv)
            print('takeoff time = %2.1f s' % takeoff_time)

        # check for apogee time
        if 'Apogée' in row:
            conv = row[self.indexes['temps']].replace(',', '.')
            self.apogee_time = float(conv)
            print('apogee time = %2.1f s' % self.apogee_time)

        # extract values
        for key, val in self.indexes.iteritems():
            conv = row[val].replace(',', '.')

            try:
                self.data[key].append(float(conv))
            except ValueError:
                if conv == '-':
                    self.data[key].append(0.0)

    def __iter__(self):
        """prepare iteration"""
        self._iteration_step = 0

        # compute longitudinal and lateral acceleration
        # respecting the sampling rate
        # and adding samples before 0 s
        data = zip(self.data['poussée'], self.data['trainée'],
                   self.data['poids'], self.data['rampe'], self.data['beta'])
        for thrust, drag, weight, pad, angle in data:
            acc_long = thrust - drag - weight * math.sin(angle)
            acc_lat = pad - weight * math.cos(angle)

            self.data['acc long'].append(acc_long)
            self.data['acc lat'].append(acc_lat)

        # compute the extra samples
        for i in range(self._extra_samples):
            time = (i - self._extra_samples) * self._sampling_rate

            self._computed_data['temps'].append(time)
            self._computed_data['acc long'].append(self.data['acc long'][0])
            self._computed_data['acc lat'].append(self.data['acc lat'][0])


        # compute the samples respecting the sampling rate
        prev_time = -self._sampling_rate
        prev_acc_long = self.data['acc long'][0]
        prev_acc_lat = self.data['acc lat'][0]

        data = zip(self.data['temps'], self.data['acc long'], self.data['acc lat'])
        for time, acc_long, acc_lat in data:
            # oversampling
            if time < prev_time + self._sampling_rate:
                continue

            # undersampling
            while time >= prev_time + self._sampling_rate:
                self._computed_data['temps'].append(prev_time)
                self._computed_data['acc long'].append(prev_acc_long)
                self._computed_data['acc lat'].append(prev_acc_lat)

                prev_time += self._sampling_rate

            prev_acc_long = acc_long
            prev_acc_lat = acc_lat

        # object is its own iterator
        return self

    def next(self):
        """to work with __iter__()"""
        i = self._iteration_step

        if i >= len(self._computed_data['temps']):
            raise StopIteration

        # prepare data set
        res = {
            'temps': self._computed_data['temps'][i],
            'acc long': self._computed_data['acc long'][i],
            'acc lat': self._computed_data['acc lat'][i],
        }

        # add noise
        noise = random.uniform(1. - self._noise, 1 + self._noise)
        res['acc long'] = (res['acc long'] + self._bias) + noise
        noise = random.uniform(1. - self._noise, 1 + self._noise)
        res['acc lat'] = (res['acc lat'] + self._bias) + noise

        self._iteration_step += 1

        return res


class Algo(object):
    """take-off and apogee detection algo"""
    def __init__(self, takeoff=2.0, apogee=0.7, averaging=3):
        self.thresholds = {
            'takeoff': takeoff, # takeoff acceleration threshold in [g]
            'apogee': apogee,  # lateral acceleration threshold in [g]
        }
        self._averaging = averaging

        self._state = self._state_on_pad
        self._avr = {
            'acc long': [],
            'acc lat': [],
        }

        print('t [s]\tacc long [g]\tacc lat [g]\tevent')

    def _state_on_pad(self, acc_long, acc_lat):
        """rocket is waiting on pad"""
        if acc_long > self.thresholds['takeoff']:
            self._state = self._state_thrusting
            return ' --> takeoff'

    def _state_thrusting(self, acc_long, acc_lat):
        """rocket has taken off"""
        if acc_long < 0:
            self._state = self._state_balistic_up
            return ' --> balistic up'

    def _state_balistic_up(self, acc_long, acc_lat):
        """rocket is climbing up"""
        if acc_lat > self.thresholds['apogee']:
            self._state = self._state_parachute
            return ' --> parachute while up'

        if acc_long > 0:
            self._state = self._state_balistic_down
            return ' --> balistic down'

    def _state_balistic_down(self, acc_long, acc_lat):
        """rocket is falling down"""
        if acc_lat > self.thresholds['apogee']:
            self._state = self._state_parachute
            return ' --> parachute while down'

    def _state_parachute(self, acc_long, acc_lat):
        """rocket is under parchute"""
        pass

    def process(self, data):
        """process the accelerations"""
        # acceleration are average on n samples
        # takeoff <=> acc long > takeoff threshold
        # apogee <=> takeoff
        #         && acc long > 0 then < 0 then > 0
        #         && acc lat > apogee threshold

        # acceleration captor is limited to +/- 16 g
        acc_long = data['acc long'] / 9.81
        if acc_long > 16.0:
            acc_long = 16.0
        if acc_long < -16.0:
            acc_long = -16.0
        self._avr['acc long'].append(acc_long)
        if len(self._avr['acc long']) > self._averaging:
            self._avr['acc long'].pop(0)

        acc_lat = abs(data['acc lat']) / 9.81
        if acc_lat > 16.0:
            acc_lat = 16.0
        self._avr['acc lat'].append(acc_lat)
        if len(self._avr['acc lat']) > self._averaging:
            self._avr['acc lat'].pop(0)

        avr_acc_long = sum(self._avr['acc long']) / len(self._avr['acc long'])
        avr_acc_lat = sum(self._avr['acc lat']) / len(self._avr['acc lat'])

        event = ''

        state_event = self._state(avr_acc_long, avr_acc_lat)
        if state_event is None:
            state_event = ''
        event += state_event

        print('%5.2f\t%6.2f\t%6.2f\t%s' %
                (
                    data['temps'],
                    acc_long,
                    acc_lat,
                    event
                )
             )


def main():
    """process data from OpenRocket export and test apogee algo"""
    if len(sys.argv) != 2:
        print('usage:')
        print('\tsimu.py <csv file>')
        sys.exit(1)

    # extract data from csv export and generate a data set
    accel = Accel(sampling_rate=0.1, extra_samples=15, noise=0.25, bias=-0.1)

    with open(sys.argv[1], 'rb') as data_fd:
        rows = csv.reader(data_fd, delimiter='\t')

        for row in rows:
            accel.process(row)

    # run the algo on the data set
    algo = Algo(takeoff=2.0, apogee=0.6, averaging=10)

    for a in accel:
        algo.process(a)


if __name__ == '__main__':
    main()
