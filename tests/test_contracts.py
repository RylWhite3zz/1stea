from allegro_probe import ProbeCommand, make_demo_scene, primitive_for_family


def test_family_to_primitive_contract() -> None:
    assert primitive_for_family("stiffness") == "poke"
    assert primitive_for_family("mass") == "heft"
    assert primitive_for_family("fill") == "shake"
    assert primitive_for_family("material") == "slide"


def test_demo_scene_has_no_answer_field() -> None:
    scene = make_demo_scene("mass", n_candidates=3, seed=0)
    payload = scene.to_dict(reveal_hidden=False)
    assert "target" not in payload
    assert len(payload["objects"]) == 3
    assert ProbeCommand("heft", target=1).target == 1
