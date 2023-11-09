use embedded_registers::register;
use uom::num_rational::Rational32;
use uom::si::rational32::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

/// The MCP9808 uses a band gap temperature sensor circuit to output analog voltage
/// proportional to absolute temperature. An internal ΔΣ ADC is used to convert
/// the analog voltage to a digital word.
///
/// The ambient temperature register bits are double-buffered. Therefore, the user
/// can access the register, while in the background, the MCP9808 performs an
/// Analog-to-Digital conversion. The temperature data from the ΔΣ ADC is loaded
/// in parallel to the TA register at t_CONV refresh rate.
///
/// In addition, the register contains three bits to reflect the alert pin state.
/// This allows the user to identify the cause of the Alert output trigger.
/// These are not affected by the status of the Alert Output Configuration
/// in the configuration register.
///
/// The three least significant temperature bits may stay `0` depending on the
/// resolution register.
#[register(address = 0b0101, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct AmbientTemperature {
    /// Whether T_A is greater than or equal to T_CRIT
    pub is_critical: bool,
    /// Whether T_A is greater than T_UPPER
    pub is_upper: bool,
    /// Whether T_A is lower than T_LOWER
    pub is_lower: bool,
    /// The ambient temperature with a resolution of 0.0625°C/LSB (1/16 °C/LSB).
    #[bondrewd(bit_length = 13)]
    pub raw_temperature: i16,
}

crate::simple_device::add_register!(super::MCP9808, AmbientTemperature, r);

impl AmbientTemperature {
    pub fn read_temperature(&self) -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(Rational32::new_raw(self.read_raw_temperature().into(), 16))
    }
}

macro_rules! define_temp_limit_register {
    ($name:ident, $address:expr, $doc:expr) => {
        #[doc = $doc]
        #[register(address = $address, mode = "rw")]
        #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
        pub struct $name {
            #[bondrewd(bit_length = 3, reserve)]
            #[allow(dead_code)]
            reserved0: u8,
            /// The temperature limit with a resolution of 0.25°C/LSB.
            #[bondrewd(bit_length = 11)]
            pub raw_temperature_limit: i16,
            #[bondrewd(bit_length = 2, reserve)]
            #[allow(dead_code)]
            reserved1: u8,
        }

        crate::simple_device::add_register!(super::MCP9808, $name, rw);

        impl $name {
            pub fn read_temperature_limit(&self) -> ThermodynamicTemperature {
                ThermodynamicTemperature::new::<degree_celsius>(
                    Rational32::new_raw(self.read_raw_temperature_limit().into(), 4).reduced(),
                )
            }

            pub fn write_temperature_limit(
                &mut self,
                temperature_limit: ThermodynamicTemperature,
            ) -> Result<(), core::num::TryFromIntError> {
                let temp = temperature_limit.get::<degree_celsius>();
                let temp: i16 = (temp * Rational32::from_integer(4)).to_integer().try_into()?;
                self.write_raw_temperature_limit(temp);
                Ok(())
            }
        }
    };
}

define_temp_limit_register!(
    TemperatureLimitUpper,
    0b0010,
    r#"
The Alert Temperature Upper Boundary Trip register (T_UPPER).
Power-Up Default for T_UPPER is 0°C

If the alerting feature is enabled in the configuration and the ambient temperature
exceeds the value specified here, the MCP9808 asserts an alert output.
"#
);

define_temp_limit_register!(
    TemperatureLimitLower,
    0b0011,
    r#"
Alert Temperature Lower Boundary Trip register (T_LOWER).
Power-Up Default for T_LOWER is 0°C

If the alerting feature is enabled in the configuration and the ambient temperature
exceeds the value specified here, the MCP9808 asserts an alert output.
"#
);

define_temp_limit_register!(
    TemperatureLimitCrit,
    0b0100,
    r#"
The Critical Temperature Trip register (T_CRIT).
Power-Up Default for T_CRIT is 0°C

If the alerting feature is enabled in the configuration and the ambient temperature
exceeds the value specified here, the MCP9808 asserts an alert output.
"#
);
