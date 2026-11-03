# WiFi Streaming Guide

This guide tells you how to stream audio from Music Assistant to a Pico 2 W that runs the DSPi.

The WiFi input is an optional build. The standard build does not include it.

## 1. What you get

- A new input source named NET.
- Two player protocols. You can use one at a time:
  - Sendspin (default).
  - Squeezelite (Slimproto).
- Audio formats: FLAC and PCM at 44.1 kHz or 48 kHz, stereo.
- Volume and mute from the Music Assistant app on a phone.

## 2. Build the firmware

Use these settings:

| Setting | Value |
|---|---|
| Board | `pico2_w` |
| Platform | `rp2350` |
| CMake option | `-DDSPI_ENABLE_WIFI=ON` |

The build option reduces the audio buffer count from 8 to 4.
The WiFi software needs the memory.

## 3. Configure the Pico

Use vendor control requests. All of them return an error on a standard build.

| Request | Function |
|---|---|
| `0x3E`, wValue `0` | Send the WiFi network name (SSID) |
| `0x3E`, wValue `1` | Send the WiFi password |
| `0x3E`, wValue `2` | Apply the settings and save them |
| `0x3E`, wValue `3` | Clear the staged password |
| `0x3E`, wValue `4` | Forget the saved WiFi settings |
| `0x3F` | Set the Music Assistant host and port |
| `0x41` | Select the protocol: 0 = Sendspin, 1 = Squeezelite |
| `0x40` | Read the network status (32 bytes) |

Do these steps:

1. Send the SSID with `0x3E` and wValue `0`.
2. Send the password with `0x3E` and wValue `1`.
3. Send the Music Assistant host with `0x3F`. Use the default port, or set one.
4. If you want Squeezelite, send `0x41` with the value `1`.
5. Apply the settings with `0x3E` and wValue `2`.
6. Select the NET input with `REQ_SET_INPUT_SOURCE` (`0xE0`). The NET value is `7`.

The settings are saved to flash after you apply them.
Every flash write stops the stream for a few seconds. The Pico then joins the network again.

Default ports:

| Protocol | Port | Path |
|---|---|---|
| Sendspin | 8927 | `/sendspin` |
| Squeezelite | 3483 | not applicable |

## 4. Set up Music Assistant

For Sendspin, use a Music Assistant version that has the Sendspin provider.
The DSPi connects to Music Assistant. Music Assistant does not need to find the DSPi.

For Squeezelite, enable the Squeezelite (Slimproto) provider in Music Assistant.
The DSPi appears as a player named DSPi.

Use the Music Assistant app to select the player and to change the volume.

## 5. Volume and mute

Music Assistant sends the volume as a percentage.
The DSPi converts the percentage to the user volume.
The lowest volume is -60 dB.

Sendspin reports local volume changes back to Music Assistant.
Squeezelite cannot do this. The next volume command from Music Assistant replaces the local value.

## 6. Limits

- The firmware was built and tested in software only. It has not run on a Pico 2 W or with a real Music Assistant server.
- The Sendspin protocol is in technical preview. It can change.
- The Sendspin device key is saved to flash. The WiFi password and the key are not encrypted.
- FLAC does not work while the ADAT or TDM8 output is on. The stream uses PCM.
- FLAC blocks must be 4608 samples or fewer. The DSPi drops larger blocks.
- 24-bit FLAC is not advertised by default.
- Squeezelite does not support timed pause, start-at-time or sync skip-ahead.
- Music Assistant sends a 200 KB buffer threshold for Squeezelite. The DSPi limits it to 24 KB.
- The audio is not synchronized with other players.

## 7. Memory

The WiFi build has very little free memory.
Do not add features that use the heap.
The build prints the heap budget. The free heap must stay above about 3 KB.
