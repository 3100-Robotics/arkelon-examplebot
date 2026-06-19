import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestTransitions {
  enum ExampleStates {
    A(21, 69),
    B,
    C,
    D;

    public int info0 = 31;
    public int info1 = 0;

    private ExampleStates(int info0, int info1) {
      this.info0 = info0;
      this.info1 = info1;
    }

    private ExampleStates() {}
  }

  @BeforeEach // this method will run before each test
  void setup() {}

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {}

  @Test
  void testCheckTransition() {
    // Transitions<ExampleStates> transition =
    //     new Transitions<ExampleStates>(ExampleStates.class)
    //         .withValidMany(ExampleStates.A, ExampleStates.B, ExampleStates.C)
    //         .withValidSelf(ExampleStates.B)
    //         .withValidAny(ExampleStates.D);

    // List<Pair<ExampleStates, ExampleStates>> checkTransitionArgsTrue =
    //     new ArrayList<>(
    //         List.of(
    //             Pair.of(ExampleStates.A, ExampleStates.B),
    //             Pair.of(ExampleStates.A, ExampleStates.C),

    //             // Test if the ecplicit self is okay
    //             Pair.of(ExampleStates.B, ExampleStates.B),

    //             // Self should always point to any as well
    //             Pair.of(ExampleStates.D, ExampleStates.D),
    //             Pair.of(ExampleStates.D, ExampleStates.A),
    //             Pair.of(ExampleStates.D, ExampleStates.B),
    //             Pair.of(ExampleStates.D, ExampleStates.C)));

    // List<Pair<ExampleStates, ExampleStates>> checkTransitionArgsFalse =
    //     new ArrayList<>(List.of(Pair.of(ExampleStates.A, ExampleStates.D)));

    // for (Pair<ExampleStates, ExampleStates> testCase : checkTransitionArgsTrue) {
    //   assert transition.checkTransition(testCase.getFirst(), testCase.getSecond());
    // }

    // for (Pair<ExampleStates, ExampleStates> testCase : checkTransitionArgsFalse) {
    //   assert !transition.checkTransition(testCase.getFirst(), testCase.getSecond());
    // }

    // for (Pair<ExampleStates, ExampleStates> testCase : checkTransitionArgsTrue) {
    //   assert transition.checkTransition(
    //       testCase.getFirst().toString(), testCase.getSecond().toString());
    // }

    // for (Pair<ExampleStates, ExampleStates> testCase : checkTransitionArgsFalse) {
    //   assert !transition.checkTransition(
    //       testCase.getFirst().toString(), testCase.getSecond().toString());
    // }
  }
}
