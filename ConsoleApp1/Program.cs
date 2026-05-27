using System;
using System.Buffers;
using System.IO.Pipelines;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.IO;
using System.Numerics;
using System.Security.Principal;

class GetImuData
{
    private UInt16 head = 0x55AA;
    private static Byte CheckSum = 16 * 15;

    private static Int16 count = 0;

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct Data
    {
        public Int16 quatC_0; public Int16 quatC_1; public Int16 quatC_2;

        public Int16 accelC_0; public Int16 accelC_1; public Int16 accelC_2;

        public Byte MaxIndex;

        public Byte McuIndex;

        public UInt16 PackNum;
    }

    public static SemaphoreSlim _semaphore = new SemaphoreSlim(1, 1);
    public static float[,] accel = new float[15, 3];
    public static float[,] quat = new float[15, 4];

    public static Data[] DataFrame = new Data[15];
    public static bool Flag = false;

    public static Stream CreatPortStream(string portName)
    {
        SerialPort port = new SerialPort(portName, 115200);
        port.DtrEnable = true;
        port.RtsEnable = true;

        port.Open();

        var stream = port.BaseStream;

        return stream;
    }

    public static async Task pipeStart()
    {
        var stream = CreatPortStream("COM6");

        var reader = PipeReader.Create(stream);
        var writer = PipeWriter.Create(Console.OpenStandardOutput());

        var processMessagesTask = ProcessMessagesAsync(reader, writer);

        await processMessagesTask;
    }

    static async Task ProcessMessagesAsync(PipeReader reader, PipeWriter writer)
    {
        try
        {
            while (true)
            {
                ReadResult result = await reader.ReadAsync();
                ReadOnlySequence<byte> buffer = result.Buffer;

                try
                {
                    if (result.IsCanceled)
                    {
                        break;
                    }

                    if (TryParseLines(ref buffer, out string message))
                    {
                        FlushResult flushResult = await WriteMessagesAsync(writer, message);
                        if (flushResult.IsCanceled || flushResult.IsCompleted)
                        {
                            break;
                        }
                    }

                    if (result.IsCompleted)
                    {
                        if (!buffer.IsEmpty)
                        {
                            throw new InvalidDataException("Incomplete message.");
                        }
                        break;
                    }
                }
                finally
                {
                    reader.AdvanceTo(buffer.Start, buffer.End);
                }
            }
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex);
        }
        finally
        {
            await reader.CompleteAsync();
            await writer.CompleteAsync();
        }
    }

    static bool TryParseLines(ref ReadOnlySequence<byte> buffer, out string message)
    {
        StringBuilder outputMessage = new();
        var reader = new SequenceReader<byte>(buffer);
        ReadOnlySpan<byte> delimiter = stackalloc byte[] { (byte)0xAA, (byte)0x55 };
        Span<byte> tempSpan = stackalloc byte[CheckSum];

        while (true)
        {
            if (!reader.TryReadTo(out ReadOnlySequence<byte> _, delimiter, advancePastDelimiter: true))
            {
                break;
            }

            if (reader.Remaining < CheckSum)
            {
                reader.Rewind(delimiter.Length);
                break;
            }

            if (reader.TryCopyTo(tempSpan))
            {
                Span<Data> decodedFrame = MemoryMarshal.Cast<byte, Data>(tempSpan);
                decodedFrame.CopyTo(DataFrame);
            }

            count++;

            _semaphore.Wait();

            for (int i = 0; i < 15; i++)
            {
                var d = DataFrame[i];

                accel[i, 0] = (float)(d.accelC_0 / 417.662);
                accel[i, 1] = (float)(d.accelC_1 / 417.662);
                accel[i, 2] = (float)(d.accelC_2 / 417.662);

                float q0 = (float)d.quatC_0 / 46340.0f;
                float q1 = (float)d.quatC_1 / 46340.0f;
                float q2 = (float)d.quatC_2 / 46340.0f;

                float sumSq = q0 * q0 + q1 * q1 + q2 * q2;
                float q3 = (float)Math.Sqrt(Math.Max(0, 1.0f - sumSq));

                if (d.MaxIndex == 0)
                {
                    quat[i, 0] = q3;
                    quat[i, 1] = q0;
                    quat[i, 2] = q1;
                    quat[i, 3] = q2;
                }
                else if (d.MaxIndex == 1)
                {
                    quat[i, 0] = q0;
                    quat[i, 1] = q3;
                    quat[i, 2] = q1;
                    quat[i, 3] = q2;
                }
                else if (d.MaxIndex == 2)
                {
                    quat[i, 0] = q0;
                    quat[i, 1] = q1;
                    quat[i, 2] = q3;
                    quat[i, 3] = q2;
                }
                else
                {
                    quat[i, 0] = q0;
                    quat[i, 1] = q1;
                    quat[i, 2] = q2;
                    quat[i, 3] = q3;
                }
            }

            Flag = true;

            _semaphore.Release();

            reader.Advance(CheckSum);
        }

        buffer = reader.UnreadSequence;

        message = outputMessage.ToString();
        return message.Length != 0;
    }
    static ValueTask<FlushResult> WriteMessagesAsync(PipeWriter writer, string message) =>
        writer.WriteAsync(Encoding.ASCII.GetBytes(message));

}