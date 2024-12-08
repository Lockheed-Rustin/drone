use drone::LockheedRustin;
use wg_2024::tests;

#[test]
fn fragment_drop() {
    tests::generic_fragment_drop::<LockheedRustin>();
}

#[test]
fn fragment_forward() {
    tests::generic_fragment_forward::<LockheedRustin>();
}

#[test]
fn chain_fragment_drop() {
    tests::generic_chain_fragment_drop::<LockheedRustin>();
}

#[test]
fn chain_fragment_ack() {
    tests::generic_chain_fragment_ack::<LockheedRustin>();
}
