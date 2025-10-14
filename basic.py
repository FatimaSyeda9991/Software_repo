import qwiic_as7265x
import sys

def runExample():
    print("\nQwiic Spectral Triad Example 1 - Basic\n")

    # Create instance of device
    myAS7265x = qwiic_as7265x.QwiicAS7265x()

    # Check if it's connected
    if myAS7265x.is_connected() == False:
        print("The device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    # Initialize the device
    if myAS7265x.begin() == False:
        print("Unable to initialize the AS7265x. Please check your connection", file = sys.stderr)
        return

    print("A,B,C,D,E,F,G,H,R,I,S,J,T,U,V,W,K,L")

    while True:
        # This is a hard wait while all 18 channels are measured
        myAS7265x.take_measurements()
        print(str(myAS7265x.get_calibrated_a()) + ",", end="")  # 410nm
        print(str(myAS7265x.get_calibrated_b()) + ",", end="")  # 435nm
        print(str(myAS7265x.get_calibrated_c()) + ",", end="")  # 460nm
        print(str(myAS7265x.get_calibrated_d()) + ",", end="")  # 485nm
        print(str(myAS7265x.get_calibrated_e()) + ",", end="")  # 510nm
        print(str(myAS7265x.get_calibrated_f()) + ",", end="")  # 535nm

        print(str(myAS7265x.get_calibrated_g()) + ",", end="")  # 560nm
        print(str(myAS7265x.get_calibrated_h()) + ",", end="")  # 585nm
        print(str(myAS7265x.get_calibrated_r()) + ",", end="")  # 610nm
        print(str(myAS7265x.get_calibrated_i()) + ",", end="")  # 645nm
        print(str(myAS7265x.get_calibrated_s()) + ",", end="")  # 680nm
        print(str(myAS7265x.get_calibrated_j()) + ",", end="")  # 705nm

        print(str(myAS7265x.get_calibrated_t()) + ",", end="")  # 730nm
        print(str(myAS7265x.get_calibrated_u()) + ",", end="")  # 760nm
        print(str(myAS7265x.get_calibrated_v()) + ",", end="")  # 810nm
        print(str(myAS7265x.get_calibrated_w()) + ",", end="")  # 860nm
        print(str(myAS7265x.get_calibrated_k()) + ",", end="")  # 900nm
        print(str(myAS7265x.get_calibrated_l()))  # 940nm	

if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example")
        sys.exit(0)