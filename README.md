# Robot Arm Trajectory Generation using LSPB
UR5 documentation for socket communication [link](https://www.siemens-pro.ru/docs/ur/scriptManual.pdf)

Run Simulation LSPB without viapoint <br>
`$ python lspb_without_via_simulation.py`

Run Simulation LSPB with viapoint <br>
`$ python lspb_with_via_simulation.py`

Run on real UR5 robot using moveL <br>
`$ python set_home_movel.py` <br>
`$ python star_movel.py`

Run on real UR5 robot using LSPB with viapoint <br>
`$ python set_home_lspb.py` <br>
`$ python star_lspb_with_via.py`

Demo Video
[link](https://drive.google.com/file/d/1t79nH-zFW1fdlBv2GE5WuU8ilDRrKLas/view?usp=sharing)

Run on real UR5 robot using LSPB with star's corner as viapoint <br>
`$ python set_home_lspb.py` <br>
`$ python star_lspb_with_via_corner_only.py`
