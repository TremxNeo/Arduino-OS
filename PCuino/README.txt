PCuino self-registering app architecture

Apps no longer use a central registration bundle or APP_* feature flags.

To add an app:
1. Generate the app .cpp and .h with App Maker.
2. Drop both files into the PCuino sketch folder.
3. Compile/upload.
4. Use `apps` to verify it and `apps <name>` to open it.

Each app contains a static AppRegistrar. The OS discovers it automatically.
Do not edit PCuino.ino, config.h, or a central app list.

Wire IDs must be unique printable characters and may not be ':'.
The core rejects duplicate or invalid IDs.

CORE_MAX_APPS is 12.
