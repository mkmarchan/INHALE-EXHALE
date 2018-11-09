(
s.waitForBoot({
	// Parameters
	var sndPath;
	// Kernels
	var diffuseEncoder, binauralDecoder;
	// Buffers
	var sounds;
	// Busses
	var decoderBus;
	// Groups
	var positionGroup, synthesisGroup, decoderGroup;
	// Synth defs
	var pulseSynth, decoderSynth;
	// Routines / sections
	var scoreRoutine, pulseBreathRoutine;

	// Parameters -------------------------------------
	sndPath = "../res/".resolveRelative;

	// Kernels
	diffuseEncoder = FoaEncoderKernel.newDiffuse;
	binauralDecoder = FoaDecoderKernel.newSpherical;
	s.sync;

	// Buffers ----------------------------------------
	sounds = Dictionary.new;
	(sndPath ++ "*").pathMatch.collect{
		arg bufferPath;
		var buffer;
		buffer = CtkBuffer.playbuf(bufferPath).load;
		sounds.put(bufferPath.basename.splitext[0], buffer);
	};

	// Busses -----------------------------------------
	decoderBus = CtkAudio.new(4);

	// Groups -----------------------------------------
	positionGroup = CtkGroup.play(server: s);
	s.sync;
	synthesisGroup = CtkGroup.play(addAction: 'after', target: positionGroup, server: s);
	s.sync;
	decoderGroup = CtkGroup.play(addAction: 'after', target: synthesisGroup, server: s);
	s.sync;

	// Synth Defs -------------------------------------
	pulseSynth = CtkSynthDef.new(\pulseSynth, {
		arg gain, rate, pulseBuffer, outBus;

		var numBufChannels;
		var out;
		var dur;
		var env;

		numBufChannels = 1;
		dur = BufDur.ir(pulseBuffer);


		env = (LFSaw.ar(-1 * rate / dur, 1) / 2.5 + 0.6).pow(2);
		out = PlayBuf.ar(numBufChannels, pulseBuffer, rate, loop: 1);
		out = out * env * gain.dbamp;
		out = FoaEncode.ar(out, diffuseEncoder);

		Out.ar(outBus, out);
	});

	decoderSynth = CtkSynthDef.new(\decoderSynth, {
		arg inBus;

		var outBus, out;

		outBus = 0;

		out = In.ar(inBus);
		out = FoaDecode.ar(out, binauralDecoder);

		Out.ar(outBus, out);
	});

	// Routines ---------------------------------------
	pulseBreathRoutine = Routine ({
		var pulseNote;
		pulseNote = pulseSynth.note(target: synthesisGroup)
		.gain_(10)
		.rate_(1.0)
		.pulseBuffer_(sounds["Pulse"])
		.outBus_(decoderBus)
		.play;
	});

	scoreRoutine = Routine({
		var decoderNote;

		decoderNote = decoderSynth.note(target: decoderGroup)
		.inBus_(decoderBus)
		.play;

		pulseBreathRoutine.play;
		5.yield;
		"Hello".postln;
	});

	scoreRoutine.play;
})
)