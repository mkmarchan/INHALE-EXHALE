(
s.options.numOutputBusChannels_(4);
s.options.memSize_(8192 * 4);
s.waitForBoot({
	// Parameters
	var sndPath, oscDefKey, soundInPort, recordingDur, jerkOSCKey;
	// Kernels
	var diffuseEncoder, omniEncoder, binauralDecoder;
	// Buffers
	var sounds, voiceRecBuf;
	// Busses
	var voiceSpatializerBus, voicePositionBus, decoderBus, indexBus;
	// Groups
	var positionGroup, synthesisGroup, spatializerGroup, decoderGroup;
	// Synth defs
	var pulseSynth, decoderSynth, spatializerSynth, positionSynth;
	var breathSynth, droneSynth;
	// Routines / sections
	var scoreRoutine, pulseBreathRoutine;

	// Parameters -------------------------------------
	sndPath = "../res/".resolveRelative;
	oscDefKey = \oscDef;
	soundInPort = 0;
	recordingDur = 60 * 60;
	jerkOSCKey = \jerkTriggerOSC;

	// Kernels ----------------------------------------
	diffuseEncoder = FoaEncoderKernel.newDiffuse;
	omniEncoder = FoaEncoderMatrix.newOmni;
	binauralDecoder = FoaDecoderKernel.newListen;
	s.sync;

	// Buffers ----------------------------------------
	sounds = Dictionary.new;
	(sndPath ++ "*").pathMatch.collect{
		arg bufferPath;
		var buffer;
		buffer = CtkBuffer.playbuf(bufferPath).load;
		sounds.put(bufferPath.basename.splitext[0], buffer);
	};

	voiceRecBuf = CtkBuffer.buffer(recordingDur * s.sampleRate, 1, s).load;

	// Busses -----------------------------------------
	voiceSpatializerBus = CtkAudio.new(4);
	indexBus = CtkControl.new(1);
	voicePositionBus = CtkControl.new(3);
	decoderBus = CtkAudio.new(4);

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

		numBufChannels = 1;
		dur = BufDur.ir(pulseBuffer);

		out = PlayBuf.ar(numBufChannels, pulseBuffer, rate, loop: 1);
		out = out * gain.dbamp;
		out = FoaEncode.ar(out, omniEncoder);

		Out.ar(outBus, out);
	});

	breathSynth = CtkSynthDef.new(\breathSynth, {
		arg gain, buffer, indexBus, outBus;

		var maxReadDelay, readDelay, grainFreq, rate;
		var signal, pos, impulse, gdur, ratePos, out;

		maxReadDelay = 2;
		readDelay = ControlDur.ir;
		grainFreq = 100;
		rate = 1.0;

		// Get the microphone input
		signal = SoundIn.ar(soundInPort);

		// record the microphone input
		RecordBuf.ar(signal, buffer);
		pos = DelayC.ar(Delay1.ar(Line.ar(0, 1, BufDur.ir(buffer))), maxReadDelay, readDelay);

		// granular synthesis calculations
		impulse = Impulse.ar(grainFreq);
		gdur = 2 / grainFreq;
		ratePos = ratePos = (pos - (max(0, rate - 1) * gdur * SampleRate.ir / BufSamples.ir(buffer))) % 1;

		// granular synthesis
		out = GrainBuf.ar(1, impulse, gdur, buffer, rate, ratePos);

		// Ambisonic encode
		out = FoaEncode.ar(out, diffuseEncoder);
		//out = FoaZoom.ar(out, pi/2, SinOsc.ar(1) * pi / 2);

		Out.ar(outBus, out);
		Out.kr(indexBus, pos);
	});

	droneSynth = CtkSynthDef.new(\droneSynth, {
		arg gain, gFreq, rate, driftRate, index, buffer, outBus;

		var out, gdur, impulse, drift;

		gdur = 2 / gFreq;
		impulse = Impulse.ar(gFreq);
		drift = LFNoise2.ar(driftRate, pi / 2, [0, 0, 0]);

		out = GrainBuf.ar(1, impulse, gdur, buffer, rate, index);
		out = FoaEncode.ar(out, diffuseEncoder);
		out = FoaZoomX.ar(out, pi / 2);
		out = FoaRTT.ar(out, drift[0], drift[1], drift[2]);
		out = out * gain.dbamp;

		Out.ar(outBus, out);
	});


	decoderSynth = CtkSynthDef.new(\decoderSynth, {
		arg inBus;

		var outBus, out;

		outBus = 0;

		out = In.ar(inBus, 4);
		out = FoaDecode.ar(out, binauralDecoder);

		Out.ar(outBus, out);
	});

	positionSynth = CtkSynthDef.new(\positionSynth, {
		arg inX, inY, inZ, inXVel, cubeRoomSideLen, outBus;

		var outX, outY, outZ, jerk, lastJerk;

		jerk = inXVel.abs > 1500;

		Poll.kr(jerk, inXVel, 'j');

		SendTrig.kr(jerk);

		#inX, inY, inZ = Lag2.kr([inX, inY, inZ], 0.05);

		// transform from [0, 1] to [-1, 1]
		#outX, outY, outZ = ([inX, inY, inZ] - 0.5) / 0.5;

		// flip the x-axis
		outX = outX * -1;

		// Scale to room size
		#outX, outY, outZ = [outX, outY, outZ] * cubeRoomSideLen;

		// translate between Leap coordinate axes to
		// traditional audio / ambisonic axes.
		Out.kr(outBus, [outZ, outX, outY]);
	});

	// Dan's spatializer
	spatializerSynth = CtkSynthDef.new(\spatializerSynth, {
		arg inbus, positionbus, outbus, width, gain = -12,
		maxMeters = 1000, reverbGain = -18;

		var pos, cartesian, src, amplitude, freq, radialDistance, amp;
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

		// Position input
		pos = In.kr(positionbus, 3);

		// calculate theta, phi and rho
		cartesian = Cartesian(pos[0], pos[1], pos[2]);
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
		chain = FoaRotate.ar(chain, theta);

		// send out signal
		Out.ar(outbus, chain + reverb)
	});

	// Routines ---------------------------------------
	pulseBreathRoutine = Routine ({
		var pulseNote, breathPositionNote, breathNote;
		var breathSpatializerNote, dronePositionNote;
		var droneSpatializerNote;

		breathPositionNote = positionSynth.note(target: positionGroup)
		.inX_(0.5)
		.inY_(0.5)
		.inZ_(0.5)
		.inXVel_(0)
		.cubeRoomSideLen_(4.57)
		.outBus_(voicePositionBus)
		.play;

		breathNote = breathSynth.note(target: synthesisGroup)
		.gain_(0)
		.buffer_(voiceRecBuf)
		.indexBus_(indexBus)
		.outBus_(voiceSpatializerBus)
		.play;

		pulseNote = pulseSynth.note(target: synthesisGroup)
		.gain_(-6)
		.rate_(1.0)
		.pulseBuffer_(sounds["Pulse2"])
		.outBus_(decoderBus)
		.play;

		breathSpatializerNote = spatializerSynth.note(target: spatializerGroup)
		.inbus_(voiceSpatializerBus)
		.positionbus_(voicePositionBus)
		.outbus_(decoderBus)
		.width_(0)
		.gain_(0)
		.maxMeters_(1000)
		.reverbGain_(-12)
		.play;

		OSCdef.new(oscDefKey, {
			arg msg;
			var handOffset = 10;
			// 0 - message name
			// 1 - has left hand? (0 or 1)
			// 2 - x position left palm
			// 3 - y position left palm
			// 4 - z position left palm
			// 5 - roll left palm (z-axis rotation)
			// 6 - pitch left palm (x-axis rotation)
			// 7 - yaw left palm (y-axis rotation)
			// 8 - x velocity of palm
			// 9 - grab strength left palm
			// 10 - switch synth? (-1, 0, 1)
			// 11 - 19, same but for the right hand

			if((msg[handOffset + 1] > 0), {
				breathPositionNote.inX_(msg[handOffset + 2]);
				breathPositionNote.inY_(msg[handOffset + 3]);
				breathPositionNote.inZ_(msg[handOffset + 4]);
				breathPositionNote.inXVel_(msg[handOffset + 8]);
			});

		}, '/LeapMotion', recvPort: 5347);

		OSCdef.new(jerkOSCKey, {
			arg msg;

			var gFreq;

			gFreq = exprand(5, 15);

			droneSynth.note(target: synthesisGroup)
			.gain_(-18)
			.gFreq_(CtkControl.env(Env([gFreq, gFreq * 16], 60, 'exp')))
			.rate_(CtkControl.env(Env([1.0, 16.0], 60, 'exp')))
			.driftRate_(0.5)
			.index_(indexBus.getSynchronous(0))
			.buffer_(voiceRecBuf)
			.outBus_(decoderBus)
			.play;
		}, '/tr');
	});

	s.sync;

	scoreRoutine = Routine({
		var decoderNote;

		decoderNote = decoderSynth.note(target: decoderGroup)
		.inBus_(decoderBus)
		.play;

		// Section 1: Pulse + Breath
		pulseBreathRoutine.play;

		// Section 2: Shepherd Swarm

		// Section 3: Comb Breath / Percussive Breath
	});

	scoreRoutine.play;
})
)