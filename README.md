# Pitchy
Pitchy is a user oscillator for NTS-1 mk II. Pitchy transposes audio signal from AUDIO-IN. You can control the amount of transpose using the NTS-1's keyboard and the shape knob.

## Oscillator parameters
Below are the descriptions and functionalities of the adjustable parameters:

### SHAPE (knob A)
Pitch. Fine-tune the pitch, in addition to a note specified with the keyboard. 
### ALT (knob B)
Input gain. Adjust the gain of the audio source.

### Grain size (SIZE)
The largest buffer size (0) holds 8192 samples, equivalent to 170.6ms, while the smallest (8) holds 32 samples, or 6.7ms.
### Delay depth(dPtH)
Controls the feedback volume of previous data in the buffer (0-100), similar to a feedback parameter in delay effects. 

### Lower pitch bend width(down)
The lowest tone for the SHAPE parameter (in semitones.)
### Upper pitch bend width(uP)
The highest tone for the SHAPE parameter (in semitones.)

### Balance of the dry/wet signals(MIX)
Adjusts the balance between the original(dry) signal and transposed(wet) signal.
### Gating the output(wet) signal(GT-o)
Controls whether the wet signal is audible or silent while no key is ON.
### Gating the input(dry) signal(GT-I)
Controls whether the dry signal is audible or silent while no key is ON.
