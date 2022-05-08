
import cv2


def show_webcam():
    cam = cv2.VideoCapture(0)
    while True:
        ret_val, img = cam.read()
        print(img.var())
    cv2.destroyAllWindows()


def main():
    show_webcam()


if __name__ == '__main__':
    main()
