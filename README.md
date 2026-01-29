## Build Command
Replace $DRIVER and $ROBOT with the driver name and robot color

```
RUSTFLAGS="--cfg driver=\"$DRIVER\" --cfg robot=\"$ROBOT\"" cargo-v5 v5 upload"
```
