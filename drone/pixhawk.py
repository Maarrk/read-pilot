from exceptions import (OSError, KeyboardInterrupt, SystemExit)
import math

from dronekit import connect, APIException


class Pixhawk:
    def __init__(self, connection_string, connection_baud):
        self.vehicle = None
        self.connection_string = connection_string
        self.connection_baud = connection_baud

    def initialize(self):
        try:
            if self.connection_baud is not None:
                self.vehicle = connect(self.connection_string, baud=self.connection_baud, wait_ready=True)
            else:
                self.vehicle = connect(self.connection_string, wait_ready=True)

        # Bad TCP connection
        # except socket.error:
        #     print 'No server exists!'
        #     return False

        # Bad TTY connection
        except OSError as e:
            print 'No serial exists! ' + e.message
            return False

        # API Error
        except APIException:
            print 'Pixhawk connection timeout!'
            return False

        except (KeyboardInterrupt, SystemExit):
            print 'Pixhawk initialization quit'
            return False

        # Other error:
        except Exception as e:
            print 'Some other pixhawk error occured: ' + str(e)
            return False

        return True
