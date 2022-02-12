import java.util.Scanner;

public class mad_libs {
    public static void main(String[] args) {
        Scanner squirtle = new Scanner(System.in);

        System.out.println("name a noun?");
        String noun = squirtle.nextLine();
        System.out.println("name a verb?");
        String verb = squirtle.nextLine();
        System.out.println("name a noun?");
        String nounnoun = squirtle.nextLine();
        System.out.println("name a adjective?");
        String adjective = squirtle.nextLine();

        System.out.println("There was once a " + noun + " who really liked to " + verb + " diamonds. " + " Also, the next door city, " + nounnoun + " was really " + adjective + ". The End" );

    }
}