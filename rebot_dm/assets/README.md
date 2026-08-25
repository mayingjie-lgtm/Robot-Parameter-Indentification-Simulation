# reBot-DM Phase 4B Asset Manifest

The Phase 4B runtime MJCF expects the following repository-local binary STL files in this directory. They must be copied byte-for-byte from the accepted DM source asset directory; they must not be regenerated from density or substituted with another reBot model.

| File | Accepted source SHA-256 |
|---|---|
| `base_link.STL` | `22641c079014fe968702393f61e7f7f1a8f80c6c1df45a61e31545f6eaff3b65` |
| `link1.STL` | `9db637a63d37bc2c6398ea5b8ba32fb54122c637d5d77dc134c8782839b654cd` |
| `link2.STL` | `def0d9d82343dccc1eff0d9d5572eca49b3c134db9807ca12302b99d153df65e` |
| `link3.STL` | `ca14d73b5d63f352051f440f4cabc3989567f25ee9d19e7dd749f2d6f0d0561d` |
| `link4.STL` | `061909364f20bb387f9ff82cb80e506af959142f9fbb4a39dc86fc8244ee768a` |
| `link5.STL` | `3a32006227c1f9dbc70e77e07055c61e3f195f1054d4d2eb480262f0873bf001` |
| `link6.STL` | `c586a7d0b8345d2cd32eab7dfd2e334aba56e83f582a6298d41bbf718f2ebef1` |
| `gripper_link.STL` | `a826e164d420a6922e8ad5ee27813f39fa809f6e4fdf39b42fd8a59e7d12e847` |
| `gripper_left.STL` | `c4e0de70f776b3573d975a6eaaa746cccbe57c2d5bb12501da8889979d3af268` |
| `gripper_right.STL` | `615b119c0bb97a032badd5882144e4a2a50c905a3e071af1fa28b83bfe9b1b65` |

Accepted source provenance during Phase 4B validation:

```text
/home/wlsea1/j_ws/src/robot_assets/description/meshes_b601_gripper/
```

`rebot_dm/rebot_dm_runtime.xml` itself refers only to `assets/<file>` through a repository-relative `meshdir`. The absolute path above is provenance only and is not a runtime dependency.
