from Application import Application
from Cam import Cam

def main():
    cam = Cam()
    while(1) :
        print("pos X :",cam._position.x)
        print("pos Y :",cam._position.y)
        print("pos Z :",cam._position.z)
        cam.Update()

    #app = Application()
    #app.Test()


if __name__ == "__main__":
    main()
