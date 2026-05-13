# dspi_poke

Generic USB vendor-command poker for a connected DSPi.

Send GET / SET for any opcode without writing Console-app UI for it — useful
while iterating on a new firmware parameter that doesn't have a host-side
control yet.

## Install

```
pip install pyusb
# macOS:
brew install libusb
```

## Use

```
./dspi_poke.py help                              # long-form usage guide
./dspi_poke.py list                              # all REQ_* opcodes
./dspi_poke.py list preamp                       # filter by substring
./dspi_poke.py get  REQ_GET_MASTER_VOLUME 4 --as float
./dspi_poke.py set  REQ_SET_MASTER_VOLUME --float -12.5
./dspi_poke.py set  REQ_SET_PREAMP_CH --wvalue 0 --float -3.0
./dspi_poke.py set  0xAB 01 80 ff 00             # raw hex when no symbol yet
```

Symbolic opcode names are parsed from `firmware/DSPi/config.h` on demand;
the script walks up from its location to find it. Pass `--config-h <path>`
if running it from outside the repo.
