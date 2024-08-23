// 4th order piecewise approximation table, requires 6*6*4 = 144 bytes.
// - Values for R0=100Ω
// - maximum error: +- 0.0001°C
// - valid range: [-200°C, 900°C]
// - no need for binary search
#[rustfmt::skip]
const LOOKUP_TABLE_R100_RESISTANCES: [f32; 6] = [
    // maximum valid resistance for the params at the same index
    61.735_252,
    103.288_26,
    215.154_63,
    303.148_65,
    377.041_17,
    404.969_5,
];

#[rustfmt::skip]
const LOOKUP_TABLE_R100_PARAMS: [[f32; 5]; 6] = [
    // polynomial params [a, b, c, d, e])
    [1.905_779e-9,   -7.117_465e-6,   0.002_669_763,    2.221_340_2, -242.010_03 ],
    [3.361_247_3e-8, -1.478_834_1e-5, 0.003_384_182_7,  2.191_016,   -241.516_31 ],
    [9.669_08e-10,    3.129_280_5e-7, 0.000_816_996_27, 2.381_973_3, -246.776_75 ],
    [1.823_480_8e-9, -4.288_475_6e-7, 0.001_060_640_2,  2.346_023_8, -244.768_16 ],
    [3.374_456_5e-9, -2.322_593_4e-6, 0.001_931_025_4,  2.167_576_8, -231.000_35 ],
    [5.344_515e-9,   -5.238_631_3e-6, 0.003_551_305_5,  1.767_021_8, -193.827_36 ],
];

/// Inverse Callendar-Van Dusen equation for R0=100Ω,
/// based on piecewise reconstruction with 4th order polynomials.
/// The maximum error is ±1e-5°C over the whole temperature range [-200°C, 900°C] with f64, and ±1e-4°C for f32.
/// but degrades rapidly beyond that range. Inputs that correspond to values outside
/// of this range will yield unpredictable outcomes.
pub fn resistance_to_temperature_r100(r: f32) -> f32 {
    // Find the valid parameter index
    let param_index = LOOKUP_TABLE_R100_RESISTANCES
        .iter()
        .position(|&max_valid_resistance| r < max_valid_resistance)
        .unwrap_or(LOOKUP_TABLE_R100_RESISTANCES.len() - 1);
    let [a, b, c, d, e] = LOOKUP_TABLE_R100_PARAMS[param_index];
    let r2 = r * r;
    let r3 = r2 * r;
    let r4 = r2 * r2;
    a * r4 + b * r3 + c * r2 + d * r + e
}

/// Callendar-Van Dusen equation for R0=100Ω
pub fn temperature_to_resistance_r100(t: f32) -> f32 {
    const R0: f32 = 100.0;
    const A: f32 = 3.9083e-3;
    const B: f32 = -5.775e-7;
    const C: f32 = -4.183e-12;
    if t < 0.0 {
        let t2 = t * t;
        let t3 = t2 * t;
        R0 * (1.0 + A * t + B * t2 + C * (t - 100.0) * t3)
    } else {
        let t2 = t * t;
        R0 * (1.0 + A * t + B * t2)
    }
}

#[cfg(test)]
mod test {
    use super::resistance_to_temperature_r100;
    use approx::assert_abs_diff_eq;

    #[test]
    #[rustfmt::skip]
    fn test_lookup_table_r100() {
        assert_abs_diff_eq!(resistance_to_temperature_r100(18.520_08), -200.0, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(23.206_966), -189.109_99, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(27.853_231), -178.219_96, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(32.462_5), -167.327_73, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(37.035_477), -156.437_71, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(41.576_435), -145.545_49, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(46.085_815), -134.655_47, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(50.567_596), -123.763_245, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(55.021_954), -112.873_22, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(59.452_55), -101.981, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(63.859_31), -91.090_98, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(68.245_6), -80.198_76, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(72.611_08), -69.308_74, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(76.958_79), -58.416_515, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(81.288_15), -47.526_497, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(85.601_91), -36.634_274, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(89.899_2), -25.744_251, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(94.182_49), -14.852_03, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(98.450_62), -3.962_008, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(102.705_765), 6.930_214, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(106.946_34), 17.820_236, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(111.174_08), 28.712_458, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(115.387_26), 39.602_478, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(119.587_6), 50.494_7, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(123.773_384), 61.384_724, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(127.946_31), 72.276_95, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(132.104_7), 83.166_97, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(136.250_23), 94.059_19, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(140.381_23), 104.949_21, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(144.499_34), 115.841_43, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(148.602_94), 126.731_45, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(152.693_66), 137.623_67, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(156.769_85), 148.513_7, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(160.833_18), 159.405_91, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(164.881_97), 170.295_94, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(168.917_88), 181.188_16, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(172.939_29), 192.078_19, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(176.947_8), 202.970_41, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(180.941_8), 213.860_43, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(184.922_91), 224.752_66, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(188.889_51), 235.642_67, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(192.843_22), 246.534_9, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(196.782_42), 257.424_93, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(200.708_72), 268.317_14, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(204.620_53), 279.207_15, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(208.519_42), 290.099_37, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(212.403_85), 300.989_4, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(216.275_34), 311.881_62, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(220.132_35), 322.771_64, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(223.976_46), 333.663_88, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(227.806_08), 344.553_9, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(231.622_76), 355.446_1, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(235.424_97), 366.336_12, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(239.214_26), 377.228_36, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(242.989_09), 388.118_38, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(246.750_96), 399.010_6, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(250.498_4), 409.900_63, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(254.232_86), 420.792_85, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(257.952_88), 431.682_86, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(261.659_97), 442.575_07, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(265.352_6), 453.465_12, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(269.032_3), 464.357_33, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(272.697_5), 475.247_34, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(276.349_8), 486.139_6, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(279.987_6), 497.029_6, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(283.612_5), 507.921_8, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(287.222_9), 518.811_8, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(290.820_37), 529.704_04, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(294.403_4), 540.594_06, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(297.973_48), 551.486_3, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(301.529_1), 562.376_34, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(305.071_78), 573.268_55, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(308.6), 584.158_57, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(312.115_26), 595.050_8, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(315.616_12), 605.940_8, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(319.103_94), 616.833, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(322.577_42), 627.723_1, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(326.037_87), 638.615_3, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(329.483_92), 649.505_3, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(332.916_96), 660.397_5, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(336.335_6), 671.287_54, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(339.741_24), 682.179_75, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(343.132_48), 693.069_76, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(346.510_74), 703.962_04, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(349.874_6), 714.852_05, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(353.225_43), 725.744_26, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(356.561_9), 736.634_3, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(359.885_3), 747.526_5, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(363.194_37), 758.416_5, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(366.490_4), 769.308_7, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(369.772_06), 780.198_7, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(373.040_7), 791.091, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(376.294_98), 801.981, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(379.536_2), 812.873_2, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(382.763_06), 823.763_24, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(385.976_87), 834.655_46, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(389.176_33), 845.545_5, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(392.362_76), 856.437_7, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(395.534_85), 867.327_76, epsilon = 0.0001);
        assert_abs_diff_eq!(resistance_to_temperature_r100(398.693_85), 878.22, epsilon = 0.0001);
    }
}
