import schedule
import time

def jobOne():
    print("RF Read")

def jobTwo():
    print("RF Write")



schedule.every(.05).seconds.do(jobOne)
schedule.every(.1).seconds.do(jobTwo)


while True :
    schedule.run_pending()
    time.sleep(.01)
