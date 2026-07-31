#include "fstream.h"
#include <algorithm>
#include <cstring>
#include <ios>
#include <vector>

// 磁盘事件类型：正常、故障、更换
enum class EventType {
  NORMAL,  // 正常：所有磁盘工作正常
  FAILED,  // 故障：指定磁盘发生故障（文件被删除）
  REPLACED // 更换：指定磁盘被更换（文件被清空）
};

class RAID5Controller {
private:
  std::vector<sjtu::fstream *> drives_;
  int blocks_per_drive_;
  int block_size_;
  int num_disks_;
  int failed_drive_;
  bool started_;
  std::vector<char> work_;
  std::vector<char> temp_;

  long long OffsetOfBlock(int stripe) const {
    return 1LL * stripe * block_size_;
  }

  int ParityDisk(int stripe) const {
    return (num_disks_ - 1 - stripe % num_disks_ + num_disks_) % num_disks_;
  }

  void DecodeBlock(int block_id, int &stripe, int &disk) const {
    stripe = block_id / (num_disks_ - 1);
    int pos = block_id % (num_disks_ - 1);
    int parity = ParityDisk(stripe);
    disk = 0;
    for (int seen = 0; seen <= pos; ++disk) {
      if (disk == parity) {
        continue;
      }
      if (seen == pos) {
        break;
      }
      ++seen;
    }
  }

  void ReadRawBlock(int drive, int stripe, char *buffer) {
    sjtu::fstream *file = drives_[drive];
    file->seekg(OffsetOfBlock(stripe), std::ios::beg);
    file->read(buffer, block_size_);
  }

  void WriteRawBlock(int drive, int stripe, const char *buffer) {
    sjtu::fstream *file = drives_[drive];
    file->seekp(OffsetOfBlock(stripe), std::ios::beg);
    file->write(buffer, block_size_);
    file->flush();
  }

  void XorInto(char *dst, const char *src) {
    for (int i = 0; i < block_size_; ++i) {
      dst[i] ^= src[i];
    }
  }

  void ReconstructBlock(int drive, int stripe, char *buffer) {
    std::fill(buffer, buffer + block_size_, 0);
    for (int d = 0; d < num_disks_; ++d) {
      if (d == drive) {
        continue;
      }
      ReadRawBlock(d, stripe, temp_.data());
      XorInto(buffer, temp_.data());
    }
  }

  void RebuildDrive(int drive) {
    for (int stripe = 0; stripe < blocks_per_drive_; ++stripe) {
      ReconstructBlock(drive, stripe, work_.data());
      WriteRawBlock(drive, stripe, work_.data());
    }
  }

public:
  RAID5Controller(std::vector<sjtu::fstream *> drives, int blocks_per_drive,
                  int block_size = 4096)
      : drives_(std::move(drives)), blocks_per_drive_(blocks_per_drive),
        block_size_(block_size), num_disks_(static_cast<int>(drives_.size())),
        failed_drive_(-1), started_(false), work_(block_size), temp_(block_size) {}

  void Start(EventType event_type_, int drive_id) {
    started_ = true;
    failed_drive_ = -1;
    if (event_type_ == EventType::FAILED) {
      failed_drive_ = drive_id;
      return;
    }
    if (event_type_ == EventType::REPLACED) {
      RebuildDrive(drive_id);
    }
  }

  void Shutdown() {
    started_ = false;
    for (sjtu::fstream *drive : drives_) {
      if (drive != nullptr && drive->is_open()) {
        drive->close();
      }
    }
  }

  void ReadBlock(int block_id, char *result) {
    int stripe, disk;
    DecodeBlock(block_id, stripe, disk);
    if (disk != failed_drive_) {
      ReadRawBlock(disk, stripe, result);
      return;
    }
    ReconstructBlock(disk, stripe, result);
  }

  void WriteBlock(int block_id, const char *data) {
    int stripe, disk;
    DecodeBlock(block_id, stripe, disk);
    int parity = ParityDisk(stripe);

    if (failed_drive_ == -1) {
      ReadRawBlock(disk, stripe, work_.data());
      ReadRawBlock(parity, stripe, temp_.data());
      for (int i = 0; i < block_size_; ++i) {
        temp_[i] ^= work_[i] ^ data[i];
      }
      WriteRawBlock(disk, stripe, data);
      WriteRawBlock(parity, stripe, temp_.data());
      return;
    }

    if (disk == failed_drive_) {
      ReconstructBlock(disk, stripe, work_.data());
      if (parity != failed_drive_) {
        ReadRawBlock(parity, stripe, temp_.data());
        for (int i = 0; i < block_size_; ++i) {
          temp_[i] ^= work_[i] ^ data[i];
        }
        WriteRawBlock(parity, stripe, temp_.data());
      }
      return;
    }

    if (parity == failed_drive_) {
      WriteRawBlock(disk, stripe, data);
      return;
    }

    ReadRawBlock(disk, stripe, work_.data());
    ReadRawBlock(parity, stripe, temp_.data());
    for (int i = 0; i < block_size_; ++i) {
      temp_[i] ^= work_[i] ^ data[i];
    }
    WriteRawBlock(disk, stripe, data);
    WriteRawBlock(parity, stripe, temp_.data());
  }

  int Capacity() {
    return (num_disks_ - 1) * blocks_per_drive_;
  }
};
