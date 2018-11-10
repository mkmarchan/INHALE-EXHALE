(
s.waitForBoot({
	// Parameters
	var sndPath;
	// Kernels
	var diffuseEncoder, binauralDecoder;
	// Buffers
	var sounds;
	// Busses
	var voiceSpatializerBus, decoderBus;
	// Groups
	var positionGroup, synthesisGroup, spatializerGroup, decoderGroup;
	// Synth defs
	var pulseSynth, decoderSynth, spatializerSynth;
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
	voiceSpatializerBus = CtkAudio.new(4);

	// Groups -----------------------------------------
	positionGroup = CtkGroup.play(server: s);
	s.sync;
	synthesisGroup = CtkGroup.play(addAction: 'after', target: positionGroup, server: s);
	s.sync;
	spatializerGroup = CtkGroup.play(addAction: 'after', target: synthesisGroup, server: s);
	s.sync;
	decoderGroup = CtkGroup.play(addAction: 'after', target: spatializerGroup, server: s);
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

	// Dan's spatializer
	spatializerSynth = CtkSynthDef('spatializerSynth', {
		arg inbus, masterX, masterY, masterZ, outbus, width, gain = -12,
		maxMeters = 1000, reverbGain = -18;

		var cartesian, src, amplitude, freq, radialDistance, amp;
		var phi, theta,  chain, hiFreqAttenCoeff, minAttenDist, lpfDB, speedOfSound;
		var reverb;

		// lets set a few variables first
		// our high frequency absorption constant
		hiFreqAttenCoeff = 100000;

		// at 15 meters our LPF freq would be 20,000, there's no point in
		// having a LPF higher than that
		minAttenDist = 15;

		// our LPF's cutoff freq is always at -3dB, so this is what freq we
		// need to calculate
		lpfDB = 3.0;

		// the speed of sound at sea level under normal conditions
		speedOfSound = 344;

		//convert gain to amplitude scalar
		amp = gain.dbamp;

		// B Format input
		src = In.ar(inbus, 4);

		// calculate theta, phi and rho
		cartesian = Cartesian(masterX, masterY, masterZ);
		theta = cartesian.theta;
		phi = cartesian.phi;
		radialDistance = cartesian.rho;

		// calculate the attenuation of high frequencies based on radial distance
		// dB/m = freq/100000
		freq = lpfDB/radialDistance.clip(minAttenDist, maxMeters) * hiFreqAttenCoeff;

		// radialDistance = VarLag.kr(radialDistance);

		// the amplitude based on a starting amplitude, amp
		amplitude = amp / radialDistance.clip(1, maxMeters);

		// lowpass the src and apply envelope and amplitude
		chain = LPF.ar(src, freq);

		// use a delay line based on the distance to simulate a doppler shift
		chain = DelayC.ar(chain, maxMeters/speedOfSound, radialDistance/speedOfSound);

		// create the reverb
		reverb = FoaReverb.ar(chain * reverbGain.dbamp, predelay: 0.0, t60low: 0.6, t60high: 0.2, diffusion: 0.5, mod: 0.2);

		// zoom in on the center of the sound-field based on the arctangent of
		// the distance and a user-defined width
		chain = FoaZoom.ar(chain * amplitude, radialDistance.atan2(width));

		// use proximity filters based on the distance
		// note: proximity boots low frequencies so we need to condition our
		// signal by using a highpass
		chain = FoaProximity.ar(HPF.ar(HPF.ar(chain, 20.0), 20.0), radialDistance + 0.1);

		// tumble and then rotate
		chain = FoaTumble.ar(chain, phi);
		chain = FoaRotate.ar(chain, theta.poll(1));

		// send out signal
		Out.ar(outbus, chain + reverb)
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