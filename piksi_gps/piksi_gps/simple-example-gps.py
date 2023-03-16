from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_BASELINE_NED
import argparse

globalLat = 0
def main():
    parser = argparse.ArgumentParser(
        description="Swift Navigation SBP Example.")
    parser.add_argument(
        "-p",
        "--port",
        default=['/dev/ttyUSB0'],  #windows
        nargs=1,
        help="specify the serial port to use.")
    args = parser.parse_args()
    
    driver = PySerialDriver(args.port[0], baud=115200)
    framer = Framer(driver.read, None, verbose=True)
    handlerObj = Handler(framer)

    testObj = handlerObj.filter(0x020A)
   
    # item1 = next(testObj)
    # print("%.4f,%.4f" % (item[0].lat, item[0].lon))
           
    with handlerObj as source:
        test = source.filter(0x020A)
        item = next(test)
        # len2 = len(test)
        print("%.4f,%.4f" % (item[0].lat, item[0].lon)
            )
    

    #Open a connection to Piksi using the default baud rate (1Mbaud)
    # with PySerialDriver(args.port[0], baud=115200) as driver:
    #     with Handler(Framer(driver.read, None, verbose=True)) as source:
    #         try:
    #             # for msg, metadata in source.filter(0x020A):
    #                 # Print out the lat and long coordinates
    #                 test = source.filter(0x020A)
    #                 item = next(test)
    #                 print("%.4f,%.4f" % (item[0].lat, item[0].lon)
    #                                           )
                    
              
    #         except KeyboardInterrupt:
    #             pass
    
    # driver = PySerialDriver(args.port[0], baud=115200)
    # framer = Framer(driver.read, None, verbose=True)
    # handler = Handler(framer)
    # handler._SBPQueueIterator
    # handler.stop
    # handler.start
    # handler.wait()
    # handler.wait_callback


    # try: 
    #     for msg, metadata in handler.filter(0x020A):
    #             # Print out the lat and long coordinates
    #             print("%.4f,%.4f" % (msg.lat, msg.lon)
    #                                     )
    # except KeyboardInterrupt:
    #     pass
    # with handler as source:
    #     try:
    #         for msg, metadata in source.filter(0x020A):
    #             # Print out the lat and long coordinates
    #             print("%.4f,%.4f" % (msg.lat, msg.lon)
    #                                         )
    
    #     except KeyboardInterrupt:
    #         pass
    # for msg, metadata in handler.filter(0x020A):
    #     # Print out the lat and long coordinates
    #     print("%.4f,%.4f" % (msg.lat, msg.lon))



if __name__ == "__main__":
    main()