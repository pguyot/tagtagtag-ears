# tagtagtag-ears
Linux driver for tagtagtag ears

Creates `/dev/ear0` (left) and `/dev/ear1` (right) to drive tagtagtag ears.

## Installation

    make
    sudo make install

This adds:

    dtoverlay=tagtagtag-ears

to /boot/config.txt

    reboot

## Usage

A single process can open `/dev/ear0` (resp. `/dev/ear1`). Any further access will fail with `EBUSY`.

The following commands are accepted:

- `'.'`             NOP
- `'+' <count>`     Move ear <count> steps forwards (count from 0 to 255)

Example:

    echo -n -e '+\x03' > /dev/ear0

will move the ear 3 steps forward

- `'-' <count>`     Move ear <count> steps backwards (count from 0 to 255)

Example:

    echo -n -e '-\x03' > /dev/ear0

will move the ear 3 steps backward

- `'>' <position>`  Move ear forward until <position> % 17 is reached. If position > 17, perform additional turns.
If position is unknown, perform a detection running forward.

Example:

    echo -n -e '>\x03' > /dev/ear0

will move the ear forward to position "3" (3 steps after missing hole).

- `'<' <position>`  Move ear backward until <position> % 17 is reached. If position > 17, perform additional turns.
If position is unknown, perform a detection running backward.

Example:

    echo -n -e '<\x03' > /dev/ear0

will move the ear forward to position "3" (3 steps after missing hole).

- `'?'`             Get position or -1 if unknown (to be read from device)

Example:

    echo -n -e '>\x03?' > /dev/ear0 && dd if=/dev/ear0 of=/dev/stdout count=1 bs=1

will output '\x03' after ear has been moved to position 3.

- `'!'`             Get position, running a position detection if required.

## Detecting user moves and blocking I/O

Detecting user moves is achieved by reading `/dev/ear*`. Read blocks until ear is moved (it will then return 'm') or a get position command is invoked.
Once a 'm' is read, it will block until an additional movement occurs. Any command clears the buffer.

Writing will block if the ear is not idle. Most command will set the ear in a non-idle state, thus '.' can be used to block until the command ends.
Compare:

     echo -n -e '+\x0A' > /dev/ear0

with:

     echo -n -e '+\x0A.' > /dev/ear0

The first line returns immediatly. The second line blocks until the ear moved the requested steps.

## Broken ears

Ears are tested on start-up (ears perform a full turn which is also used to determine ear position).
If, at any time, no rising GPIO interrupt is received within 3 seconds, the ear is considered broken.
Any further write will fail.
Reading will return EOF.
