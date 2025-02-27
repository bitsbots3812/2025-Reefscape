package frc.robot.common;

/** Create custom contraints here */
public class ConstraintClasses {


    // Not sure how useful this one will be for future robots, but it is here if it is needed. 
    // In reality, this can be any value, not just pitch. So either rename it to something more generic or
    // just use it as is and know that is multipurpose
    //
    // Update 21 Feb. 2025 - Renamed to more general RangeConstraint. Added method to clamp values to the range.
    public static class RangeConstraint {

        private final double UPPER_CONSTRAINT;
        private final double LOWER_CONSTRAINT;
    
        /**
         * @param LOWER_CONSTRAINT
         * @param UPPER_CONSTRAINT
         */
        public RangeConstraint(double LOWER_CONSTRAINT, double UPPER_CONSTRAINT) {
          this.LOWER_CONSTRAINT = LOWER_CONSTRAINT;
          this.UPPER_CONSTRAINT = UPPER_CONSTRAINT;
        }
    
        /**
         * @return Returns Lower Constraint of a pitch constraint
         */
        public double getLowerConstraint() {
            return LOWER_CONSTRAINT;
          }

        /**
         * @return Returns Upper Constraint of a pitch constraint
         */
        public double getUpperConstraint() {
          return UPPER_CONSTRAINT;
        }

        /**
        * @param input
        * @return Returns the input value if it is within the constraints, otherwise returns the closest constraint.
        */
        public double clamp(double input) {
          if (input > LOWER_CONSTRAINT) {
            if (input < UPPER_CONSTRAINT) {
              return input;
            }
            else {
              return UPPER_CONSTRAINT;
            }
          }
          else {
            return LOWER_CONSTRAINT;
          }
          
        }

        /**
         * 
         * @param value
         * @return Retruns true if value is within the constrained range.
         */
        public boolean inRange(double value) {
          return (value <= UPPER_CONSTRAINT) && (value >= LOWER_CONSTRAINT);
        }
      }
}