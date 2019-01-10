#ifndef __OGNCONV_H__
#define __OGNCONV_H__

#include <stdint.h>

uint32_t FeetToMeters(uint32_t Altitude);                             //
uint32_t MetersToFeet(uint32_t Altitude);                             //

uint16_t EncodeUR2V8(uint16_t Value);                                 // Encode unsigned 12bit (0..3832) as 10bit
uint16_t DecodeUR2V8(uint16_t Value);                                 // Decode 10bit 0..0x3FF

uint8_t EncodeUR2V5(uint16_t Value);                                  // Encode unsigned 9bit (0..472) as 7bit
uint16_t DecodeUR2V5(uint16_t Value);                                 // Decode 7bit as unsigned 9bit (0..472)

uint8_t EncodeSR2V5(int16_t Value);                                   // Encode signed 10bit (-472..+472) as 8bit
int16_t DecodeSR2V5( int16_t Value);                                  // Decode

uint16_t EncodeUR2V6(uint16_t Value);                                 // Encode unsigned 10bit (0..952) as 8 bit
uint16_t DecodeUR2V6(uint16_t Value);                                 // Decode 8bit as unsigned 10bit (0..952)

uint16_t EncodeSR2V6(int16_t Value);                                  // Encode signed 11bit (-952..+952) as 9bit
 int16_t DecodeSR2V6( int16_t Value);                                 // Decode 9bit as signed 11bit (-952..+952)

// uint16_t EncodeUR2V12(uint16_t Value);                                // encode unsigned 16-bit (0..61432) as 14-bit
// uint16_t DecodeUR2V12(uint16_t Value);

// uint8_t EncodeUR2V4(uint8_t DOP);
// uint8_t DecodeUR2V4(uint8_t DOP);

template <class Type, int Bits>
 Type UnsVRdecode(Type Value)
{ const Type Thres = 1<<Bits;
  uint8_t Range = Value>>Bits;
  Value &= Thres-1;
  if(Range==0) return            Value;
  if(Range==1) return   Thres+1+(Value<<1);
  if(Range==2) return 3*Thres+2+(Value<<2);
               return 7*Thres+4+(Value<<3); }

template <class Type, int Bits>
 Type UnsVRencode(Type Value)
{ const Type Thres = 1<<Bits;
  if(Value<   Thres) return             Value;
  if(Value< 3*Thres) return   Thres | ((Value-  Thres)>>1);
  if(Value< 7*Thres) return 2*Thres | ((Value-3*Thres)>>2);
  if(Value<15*Thres) return 3*Thres | ((Value-7*Thres)>>3);
                     return 4*Thres-1; }

template <class Type, int Bits>
 Type SignVRencode(Type Value)
{ const Type SignMask = 1<<(Bits+2);
  Type Sign=0; if(Value<0) { Value=(-Value); Sign=SignMask; }
  Value = UnsVRencode<Type, Bits>(Value);
  return Value | Sign; }

template <class Type, int Bits>
 Type SignVRdecode(Type Value)
{ const Type SignMask = 1<<(Bits+2);
  Type Sign = Value&SignMask;
  Value = UnsVRdecode<Type, Bits>(Value&(SignMask-1));
  return Sign ? -Value: Value; }

uint8_t EncodeGray(uint8_t Binary);
uint8_t DecodeGray(uint8_t Gray);
uint16_t EncodeGray(uint16_t Binary);
uint16_t DecodeGray(uint16_t Gray);
uint32_t EncodeGray(uint32_t Binary);
uint32_t DecodeGray(uint32_t Gray);

void TEA_Encrypt (uint32_t* Data, const uint32_t *Key, int Loops);
void TEA_Decrypt (uint32_t* Data, const uint32_t *Key, int Loops);

void TEA_Encrypt_Key0 (uint32_t* Data, int Loops);
void TEA_Decrypt_Key0 (uint32_t* Data, int Loops);

void XorShift32(uint32_t &Seed);      // simple random number generator
void xorshift64(uint64_t &Seed);

#endif // __OGNCONV_H__
