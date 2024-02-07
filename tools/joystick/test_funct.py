import random

def get_simulated_real_speed(target_speed):
        """
        pretend real speed : take a random number [-1, 1] and divide by 20 and add 1, multiply this by the current target speed.
        """
        # not sure how well this would simulate lowering speeds as its harder to get negative nums
        #rand_num = random.randint(-100, 100)
        rand_num = random.randint(-20, 20)
        rand_num /= 1000
        rand_num *= 10
        print("factor of:", rand_num)
        adjusted_speed = (target_speed * rand_num) + target_speed
        print("Adjusted speed:", adjusted_speed)
        return adjusted_speed

print(get_simulated_real_speed(20))