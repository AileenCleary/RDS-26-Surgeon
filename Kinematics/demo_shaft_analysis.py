import config as cfg
from shaft_analysis import going_insane

def main():
    pulleys = cfg.ALL_PULLEYS
    tendons = cfg.TENDONS

    MCP_ext = tendons["MCP_extensor_demo"]
    PIP_flex = tendons["PIP_flexor_demo"]

    for name, tendon in tendons.items():
        going_insane(
            tendon=tendon,
            pulleys=pulleys,
            T0=50.0,
            mu=0.05,
            friction_mode="decay",
            use_tan_wrap=True,
            use_tan_force=True,
        )

if __name__ == "__main__":
    main()