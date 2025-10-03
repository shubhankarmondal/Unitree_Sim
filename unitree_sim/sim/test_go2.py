import mujoco
import mujoco.viewer

def main():
    # Load the Go2 XML
    model = mujoco.MjModel.from_xml_path("unitree_sim/robots/go2/scene.xml")
    data = mujoco.MjData(model)

    print("✅ Loaded Unitree Go2 model")

    # Launch viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()

if __name__ == "__main__":
    main()
