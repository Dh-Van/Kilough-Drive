/**
 * FizzBuzzSolution
 */
public class Main{

    public static void main(String[] args) {
        int counter = 0;

        while(counter < 1000){
            if(counter % 3 == 0 && counter % 5 == 0){
                System.out.println("FizzBuzz");
            } else if(counter % 3 == 0){
                System.out.println("Fizz");
            } else if(counter % 5 == 0){
                System.out.println("Buzz");
            } else {
                System.out.println(counter);
            }
            counter++;
        }

    }
}