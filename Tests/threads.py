import threading
import time

done = False


def func1():
    while not done:
        time.sleep(1)
        print("func1")


def main():
    t1 = threading.Thread(target=func1, daemon=True)
    t1.start()

    while True:
        time.sleep(1)
        print("aux")


if __name__ == "__main__":
    main()
