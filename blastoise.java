import java.util.Scanner;

public class blastoise {
    public static void main(String[] args) {
        Scanner squirtle = new Scanner(System.in);

        System.out.println("What is your last name ?");
        String lastName = squirtle.nextLine();
        System.out.println("What is your first name ?");
        String firstName = squirtle.nextLine();

        System.out.println("Your school email is : " + firstName.toLowerCase() + lastName.toLowerCase() + "@gmail.com");

    }
}