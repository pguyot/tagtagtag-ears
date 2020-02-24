# Test procedures

## Test initial detection

### At module start-up, ears should be detected properly when placed vertically

1. Move ears vertically.

2. Load module.

```
sudo insmod /lib/modules/*/kernel/input/misc/tagtagtag-ears.ko
sudo chmod ugo+rw /dev/ear*
```

3. Check positions

```
echo -n -e '?' > /dev/ear0 && dd if=/dev/ear0 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
echo -n -e '?' > /dev/ear1 && dd if=/dev/ear1 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
```

Should be 0 and 0.

4. Unload module

```
sudo rmmod tagtagtag_ears
```

### At module start-up, ears should be detected properly when placed horizontally

1. Move ears horizontally.

2. Load module.

```
sudo insmod /lib/modules/*/kernel/input/misc/tagtagtag-ears.ko
sudo chmod ugo+rw /dev/ear*
```

3. Check positions

```
echo -n -e '?' > /dev/ear0 && dd if=/dev/ear0 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
echo -n -e '?' > /dev/ear1 && dd if=/dev/ear1 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
```

Should be 10 and 10.

4. Unload module

```
sudo rmmod tagtagtag_ears
```

## Test moves

### 0 is vertical, 10 is horizontal

1. Move ears anywhere but vertically.

2. Load module.

```
sudo insmod /lib/modules/*/kernel/input/misc/tagtagtag-ears.ko
sudo chmod ugo+rw /dev/ear*
```

3. Move forward to 0, 10 and 0

```
echo -n -e '>\x00' > /dev/ear0
echo -n -e '>\x00' > /dev/ear1
```

```
echo -n -e '>\x0A' > /dev/ear0
echo -n -e '>\x0A' > /dev/ear1
```

```
echo -n -e '>\x00' > /dev/ear0
echo -n -e '>\x00' > /dev/ear1
```

4. Move backward to 10, 0 and 10

```
echo -n -e '<\x0A' > /dev/ear0
echo -n -e '<\x0A' > /dev/ear1
```

```
echo -n -e '<\x00' > /dev/ear0
echo -n -e '<\x00' > /dev/ear1
```

```
echo -n -e '<\x0A' > /dev/ear0
echo -n -e '<\x0A' > /dev/ear1
```

5. Unload module

```
sudo rmmod tagtagtag_ears
```

### Gap is between 13 and 14

1. Move ears horizontally.

2. Load module.

```
sudo insmod /lib/modules/*/kernel/input/misc/tagtagtag-ears.ko
sudo chmod ugo+rw /dev/ear*
```

3. Move forward to 13, then 14, then back to 13 and 14

```
echo -n -e '>\x0D' > /dev/ear0
echo -n -e '>\x0D' > /dev/ear1
```

```
echo -n -e '>\x0E' > /dev/ear0
echo -n -e '>\x0E' > /dev/ear1
```

```
echo -n -e '<\x0D' > /dev/ear0
echo -n -e '<\x0D' > /dev/ear1
```

```
echo -n -e '>\x0E' > /dev/ear0
echo -n -e '>\x0E' > /dev/ear1
```

4. Physically move them between 13 and 14 (will start high), then ask them to move back to 13.

```
echo -n -e '<\x0D' > /dev/ear0
echo -n -e '<\x0D' > /dev/ear1
```

Will run a full detection.

5. Physically move them between 13 and 14 (will start high), then ask them to move back to 14.

```
echo -n -e '>\x0E' > /dev/ear0
echo -n -e '>\x0E' > /dev/ear1
```

Will run a full detection.

6. Unload module

```
sudo rmmod tagtagtag_ears
```


## Test detection

### Test detection after a move

1. Move ears vertically.

2. Load module.

```
sudo insmod /lib/modules/*/kernel/input/misc/tagtagtag-ears.ko
sudo chmod ugo+rw /dev/ear*
```

3. Move ears and put them back vertically

4. Get ear movement byte

```
dd if=/dev/ear0 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
dd if=/dev/ear1 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
```

Will print 109 ('m').

5. Check ears do not know where they are

```
echo -n -e '?' > /dev/ear0 && dd if=/dev/ear0 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
echo -n -e '?' > /dev/ear1 && dd if=/dev/ear1 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
```

Should return -1 and -1.

6. Run detection

```
echo -n -e '!' > /dev/ear0 && dd if=/dev/ear0 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
echo -n -e '!' > /dev/ear1 && dd if=/dev/ear1 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'
```

Should run detection and return ears to vertical positions, returning 0 and 0.

7. Unload module

```
sudo rmmod tagtagtag_ears
```

