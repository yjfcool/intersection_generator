#pragma once
/**
 * 轻量级 Shapefile 写入器（自实现）
 * 支持输出 POLYLINE(3) 和 POLYGON(5) 类型的 Shapefile
 * 格式参考：ESRI Shapefile Technical Description
 * 无需任何外部依赖
 */
#include "../core/data_types.h"
#include <fstream>
#include <string>
#include <vector>
#include <cstring>
#include <filesystem>
#include <cmath>
#include <iostream>

class ShapefileWriter {
public:
    // Shapefile 几何类型
    static constexpr int SHP_NULL    = 0;
    static constexpr int SHP_POINT   = 1;
    static constexpr int SHP_POLYLINE = 3;
    static constexpr int SHP_POLYGON  = 5;

    struct Field {
        char   name[11];   // 字段名
        char   type;       // 'C'=字符串, 'N'=数值, 'F'=浮点
        int    length;     // 字段长度
        int    decimals;   // 小数位数
    };

    struct Record {
        std::vector<std::string> values;  // 与 fields 对应
        Polyline                 geom;    // 几何（折线或多边形）
        bool                     isPolygon = false;
    };

    // 写出一组记录到 Shapefile（同时生成 .shp/.shx/.dbf/.prj 四个文件）
    static bool write(
        const std::string& outputDir,   // 输出目录
        const std::string& filename,   // 不含扩展名的文件名称
        int shapeType,                 // SHP_POLYLINE 或 SHP_POLYGON
        const std::vector<Field>& fields,
        const std::vector<Record>& records)
    {
        bool ok = true;
        ok &= writeShp(outputDir+"/"+filename+".shp", outputDir+"/"+filename+".shx", shapeType, records);
        ok &= writeDbf(outputDir+"/"+filename+".dbf", fields, records);
        ok &= writePrj(outputDir+"/"+filename+".prj");
        return ok;
    }

    // ==================== 高层接口 ====================

    static bool writeIntersectionInput(
        const std::string& dir,
        const IntersectionInput& inp,
        const std::string& prefix = "input") {
        ShapefileWriter::writeCenterlines(dir, prefix + "_centerlines", inp.centerlines);
        ShapefileWriter::writeEdgelines(dir, prefix + "_edgelines", inp.edgelines);
        ShapefileWriter::writeObstacles(dir, prefix + "_obstacles", inp.obstacles);
        ShapefileWriter::writeStopLines(dir, prefix + "_stoplines", inp.stopLines);
        ShapefileWriter::writeRoadEdges(dir, prefix + "_roadedges", inp.roadEdges);
        return true;
    }
    static void writeCenterlines(const std::string& dir, const std::string& fname,
        const std::map<std::string, LaneCenterline>& centerlines){
        std::filesystem::create_directories(dir);
        std::vector<Field> fields = {
            makeField("ID",       'C', 64, 0),
            makeField("LANE_ORDER",  'C', 64, 0),
            makeField("GROUP_ID",'C', 64, 0),
            makeField("GROUP_TYPE",'C', 64, 0),
            makeField("LEFT_EID",'C', 64, 0),
            makeField("RIGHT_EID",'C', 64, 0),
        };
        std::vector<Record> records;
        for(auto idcl : centerlines){
            auto cl = idcl.second;
            Record r;
            r.geom = cl.geom;
            r.isPolygon = false;
            r.values = {
                cl.id,
                std::to_string(cl.laneOrder),
                cl.groupId,
                cl.groupType == GroupType::ENTER ? "enter" : "exit",
                cl.leftEdgelineId,
                cl.rightEdgelineId
            };
            records.push_back(r);
        }
        write(dir, fname, SHP_POLYLINE, fields, records);
        std::cout << "[INFO]  Shapefile written: " << dir << "/" << fname << ".*\n";
    }
    static void writeEdgelines(const std::string& dir, const std::string& fname,
        const std::map<std::string, LaneEdgeLine>& edgelines){
        std::filesystem::create_directories(dir);
        std::vector<Field> fields = {
            makeField("ID",       'C', 64, 0),
            makeField("GROUP_ID",'C', 64, 0),
            makeField("GROUP_TYPE",'C', 64, 0),
            makeField("LINE_ORDER",'C', 64, 0),
        };
        std::vector<Record> records;
        for(auto idel : edgelines){
            auto el = idel.second;
            Record r;
            r.geom = el.geom;
            r.isPolygon = false;
            r.values = {
                el.id,
                el.groupId,
                el.groupType == GroupType::ENTER ? "enter" : "exit",
                std::to_string(el.lineOrder)
            };
            records.push_back(r);
        }
        write(dir, fname, SHP_POLYLINE, fields, records);
        std::cout << "[INFO]  Shapefile written: " << dir << "/" << fname << ".*\n";
    }
    static void writeObstacles(const std::string& dir, const std::string& fname,
        const std::vector<Obstacle>& obstacles){
        std::filesystem::create_directories(dir);
        std::vector<Field> fields = {
            makeField("ID",       'C', 64, 0),
            makeField("GEO_TYPE",'C', 64, 0),
        };
        std::vector<Record> records;
        for(auto& obs : obstacles){
            Record r;
            r.geom = obs.geom;
            r.isPolygon = true;
            r.values = {
                obs.id,
                std::to_string((int)obs.geomType),
            };
            records.push_back(r);
        }
        write(dir, fname, SHP_POLYGON, fields, records);
        std::cout << "[INFO]  Shapefile written: " << dir << "/" << fname << ".*\n";
    }
    static void writeStopLines(const std::string& dir, const std::string& fname,
        const std::vector<StopLine>& stoplines){
        std::filesystem::create_directories(dir);
        std::vector<Field> fields = {
            makeField("ID",       'C', 64, 0),
            makeField("ENTRY_GROUP_ID",'C', 64, 0),
        };
        std::vector<Record> records;
        for(auto& sl : stoplines){
            Record r;
            r.geom = sl.geom;
            r.isPolygon = false;
            r.values = {
                sl.id,
                sl.enterGroupId,
            };
            records.push_back(r);
        }
        write(dir, fname, SHP_POLYLINE, fields, records);
        std::cout << "[INFO]  Shapefile written: " << dir << "/" << fname << ".*\n";
    }
    static void writeRoadEdges(const std::string& dir, const std::string& fname,
        const std::vector<RoadEdgeLine>& roadedges){
        std::filesystem::create_directories(dir);
        std::vector<Field> fields = {
            makeField("ID",       'C', 64, 0),
            makeField("SIDE_IDX",'C', 64, 0),
        };
        std::vector<Record> records;
        for(auto& re : roadedges){
            Record r;
            r.geom = re.geom;
            r.isPolygon = false;
            r.values = {
                re.id,
                std::to_string(re.intersectionSideIdx),
            };
            records.push_back(r);
        }
        write(dir, fname, SHP_POLYLINE, fields, records);
        std::cout << "[INFO]  Shapefile written: " << dir << "/" << fname << ".*\n";
    }





    static bool writeIntersectionOutput(
        const std::string& dir,
        const IntersectionOutput& out,
        const std::string& prefix = "output") {
        ShapefileWriter::writeGeneratedCenterlines(dir, prefix + "_centerlines", out.centerlines);
        ShapefileWriter::writeGeneratedEdgelines  (dir, prefix + "_edgelines", out.edgelines);
        ShapefileWriter::writeGeneratedPolygon    (dir, prefix + "_polygon", out.scenarioId, out.intersectionPolygon);
        return true;
    }

    static void writeGeneratedCenterlines(
        const std::string& dir,
        const std::string& fname,
        const std::vector<GeneratedCenterline>& cls)
    {
        std::filesystem::create_directories(dir);

        std::vector<Field> fields = {
            makeField("ID",       'C', 64, 0),
            makeField("CONN_ID",  'C', 64, 0),
            makeField("ENTER_LID",'C', 64, 0),
            makeField("EXIT_LID", 'C', 64, 0),
            makeField("TURN_TYPE",'C', 20, 0),
            makeField("QUALITY",  'N',  6, 0),
            makeField("WARN_DESC",'C',200, 0),
        };

        std::vector<Record> records;
        for(auto& cl : cls){
            Record r;
            r.geom = cl.geom;
            r.isPolygon = false;
            r.values = {
                cl.id,
                cl.connectionId,
                cl.enterLineId,
                cl.exitLineId,
                turnTypeStr(cl.turnType),
                std::to_string(cl.qualityFlags),
                cl.warnDesc.substr(0, 199)
            };
            records.push_back(r);
        }
        write(dir, fname, SHP_POLYLINE, fields, records);
        std::cout << "[INFO]  Shapefile written: " << dir << "/" << fname << ".*\n";
    }

    static void writeGeneratedEdgelines(
        const std::string& dir,
        const std::string& fname,
        const std::vector<GeneratedEdgeLine>& els)
    {
        std::filesystem::create_directories(dir);

        std::vector<Field> fields = {
            makeField("ID",        'C', 64, 0),
            makeField("LEFT_CL",   'C', 64, 0),
            makeField("RIGHT_CL",  'C', 64, 0),
            makeField("QUALITY",   'N',  6, 0),
        };

        std::vector<Record> records;
        for(auto& el : els){
            Record r;
            r.geom = el.geom;
            r.isPolygon = false;
            r.values = {
                el.id,
                el.leftCenterlineId,
                el.rightCenterlineId,
                std::to_string(el.qualityFlags)
            };
            records.push_back(r);
        }
        write(dir, fname, SHP_POLYLINE, fields, records);
        std::cout << "[INFO]  Shapefile written: " << dir << "/" << fname << ".*\n";
    }

    static void writeGeneratedPolygon(
        const std::string& dir,
        const std::string& fname,
        const std::string& scenarioId,
        const IntersectionPolygon& poly)
    {
        std::filesystem::create_directories(dir);

        std::vector<Field> fields = {
            makeField("ID",       'C', 64, 0),
            makeField("SCENARIO", 'C', 64, 0),
            makeField("AREA",     'F', 16, 4),
            makeField("QUALITY",  'N',  6, 0),
        };

        std::vector<Record> records;
        Record r;
        r.geom = poly.geom;
        r.isPolygon = true;
        r.values = {
            poly.id,
            scenarioId,
            std::to_string(poly.area),
            std::to_string(poly.qualityFlags)
        };
        records.push_back(r);

        write(dir, fname, SHP_POLYGON, fields, records);
        std::cout << "[INFO]  Shapefile written: " << dir << "/" << fname << ".*\n";
    }

private:
    static Field makeField(const char* name, char type, int len, int dec){
        Field f{};
        std::strncpy(f.name, name, 10);
        f.name[10] = '\0';
        f.type     = type;
        f.length   = len;
        f.decimals = dec;
        return f;
    }

    // ---- 字节序工具 ----
    static void writeInt32BE(std::ostream& os, int32_t v){
        uint8_t b[4];
        b[0]=(v>>24)&0xFF; b[1]=(v>>16)&0xFF;
        b[2]=(v>> 8)&0xFF; b[3]=(v    )&0xFF;
        os.write(reinterpret_cast<char*>(b),4);
    }
    static void writeInt32LE(std::ostream& os, int32_t v){
        os.write(reinterpret_cast<char*>(&v),4);
    }
    static void writeDouble(std::ostream& os, double v){
        os.write(reinterpret_cast<char*>(&v),8);
    }
    static int32_t readInt32BE(const uint8_t* b){
        return (b[0]<<24)|(b[1]<<16)|(b[2]<<8)|b[3];
    }

    // ---- SHP/SHX 写入 ----
    static bool writeShp(
        const std::string& shpPath,
        const std::string& shxPath,
        int shapeType,
        const std::vector<Record>& records)
    {
        std::ofstream shp(shpPath, std::ios::binary);
        std::ofstream shx(shxPath, std::ios::binary);
        if(!shp||!shx){ 
            std::cerr<<"[ERROR] Cannot open shp/shx: "<<shpPath<<"\n"; 
            return false; 
        }

        // 占位写头部（后续回填）
        writeShpHeader(shp, shapeType, 0, records);
        writeShpHeader(shx, shapeType, 0, records);

        int shxOffset = 50; // 单位：16-bit words，头部100字节=50 words

        std::vector<std::pair<int,int>> shxIndex; // (offset, contentLen) in 16-bit words

        for(int recNo=0; recNo<(int)records.size(); ++recNo){
            const Record& rec = records[recNo];
            const Polyline& pts = rec.geom;

            // 计算记录内容长度（字节）
            int numPts  = (int)pts.size();
            int numParts= 1;
            // 内容: shapeType(4) + bbox(32) + numParts(4) + numPoints(4) + parts[](4) + points[](16*n)
            int contentBytes = 4+32+4+4+numParts*4+numPts*16;
            int contentWords = contentBytes / 2;

            // SHX 记录
            shxIndex.push_back({shxOffset, contentWords});
            shxOffset += 4 + contentWords; // 记录头(4 words) + 内容

            // SHP 记录头（大端）
            writeInt32BE(shp, recNo+1);       // record number (1-based)
            writeInt32BE(shp, contentWords);  // content length in 16-bit words

            if(pts.empty()){
                // 空几何
                writeInt32LE(shp, SHP_NULL);
                continue;
            }

            // 包围盒
            double minX=1e18,minY=1e18,maxX=-1e18,maxY=-1e18;
            for(auto& p:pts){
                minX=std::min(minX,p.x); minY=std::min(minY,p.y);
                maxX=std::max(maxX,p.x); maxY=std::max(maxY,p.y);
            }

            // 内容（小端）
            writeInt32LE(shp, shapeType);
            writeDouble(shp, minX); writeDouble(shp, minY);
            writeDouble(shp, maxX); writeDouble(shp, maxY);
            writeInt32LE(shp, numParts);
            writeInt32LE(shp, numPts);
            writeInt32LE(shp, 0); // parts[0] = 0
            for(auto& p:pts){
                writeDouble(shp, p.x);
                writeDouble(shp, p.y);
            }
        }

        // 回填头部
        auto shpSize = shp.tellp();
        auto shxSize = shx.tellp();
        shp.seekp(0);
        shx.seekp(0);
        writeShpHeaderFill(shp, shapeType, (int)(shpSize/2), records);
        writeShxRecords(shx, shapeType, (int)(shxSize/2), records, shxIndex);

        return true;
    }

    static void writeShpHeader(std::ostream& os, int shapeType, int fileLen,
                                const std::vector<Record>& records)
    {
        // 占位，写100字节头
        writeInt32BE(os, 9994);          // file code
        for(int i=0;i<5;i++) writeInt32BE(os,0); // unused
        writeInt32BE(os, fileLen);       // file length (16-bit words)
        writeInt32LE(os, 1000);          // version
        writeInt32LE(os, shapeType);

        // 全局包围盒（先填0，后面不回填了，用bbox即可）
        double bbox[8]={0,0,0,0,0,0,0,0};
        for(int i=0;i<8;i++) writeDouble(os, bbox[i]);
    }

    static void writeShpHeaderFill(std::ostream& os, int shapeType, int fileLenWords,
                                    const std::vector<Record>& records)
    {
        double minX=1e18,minY=1e18,maxX=-1e18,maxY=-1e18;
        for(auto& r:records){
            for(auto& p:r.geom){
                minX=std::min(minX,p.x); minY=std::min(minY,p.y);
                maxX=std::max(maxX,p.x); maxY=std::max(maxY,p.y);
            }
        }
        if(minX>maxX){minX=maxX=minY=maxY=0;}

        writeInt32BE(os, 9994);
        for(int i=0;i<5;i++) writeInt32BE(os,0);
        writeInt32BE(os, fileLenWords);
        writeInt32LE(os, 1000);
        writeInt32LE(os, shapeType);
        writeDouble(os, minX); writeDouble(os, minY);
        writeDouble(os, maxX); writeDouble(os, maxY);
        writeDouble(os, 0);    writeDouble(os, 0);  // Z range
        writeDouble(os, 0);    writeDouble(os, 0);  // M range
    }

    static void writeShxRecords(std::ostream& os, int shapeType, int fileLenWords,
                                 const std::vector<Record>& records,
                                 const std::vector<std::pair<int,int>>& idx)
    {
        writeShpHeaderFill(os, shapeType, 50 + (int)idx.size()*4, records);
        for(auto& [offset, len] : idx){
            writeInt32BE(os, offset);
            writeInt32BE(os, len);
        }
    }

    // ---- DBF 写入（dBASE III+格式）----
    static bool writeDbf(
        const std::string& path,
        const std::vector<Field>& fields,
        const std::vector<Record>& records)
    {
        std::ofstream f(path, std::ios::binary);
        if(!f){ std::cerr<<"[ERROR] Cannot open dbf: "<<path<<"\n"; return false; }

        int numRec  = (int)records.size();
        int numFld  = (int)fields.size();
        int hdrSize = 32 + numFld*32 + 1; // 头部+字段描述+终止符
        int recSize = 1; // 删除标记
        for(auto& fd:fields) recSize += fd.length;

        // 文件头 (32字节)
        uint8_t hdr[32]={};
        hdr[0]=0x03; // dBASE III
        hdr[1]=125; hdr[2]=1; hdr[3]=1; // 日期（年月日，2025/1/1）
        // numRec (LE)
        hdr[4]=(numRec)&0xFF; hdr[5]=(numRec>>8)&0xFF;
        hdr[6]=(numRec>>16)&0xFF; hdr[7]=(numRec>>24)&0xFF;
        // headerSize (LE)
        hdr[8]=hdrSize&0xFF; hdr[9]=(hdrSize>>8)&0xFF;
        // recordSize (LE)
        hdr[10]=recSize&0xFF; hdr[11]=(recSize>>8)&0xFF;
        f.write(reinterpret_cast<char*>(hdr),32);

        // 字段描述子（每个32字节）
        for(auto& fd:fields){
            uint8_t fdesc[32]={};
            std::memcpy(fdesc, fd.name, 11);
            fdesc[11]=fd.type;
            fdesc[16]=(uint8_t)fd.length;
            fdesc[17]=(uint8_t)fd.decimals;
            f.write(reinterpret_cast<char*>(fdesc),32);
        }
        f.put(0x0D); // 头部终止符

        // 记录
        std::string buf;
        for(auto& rec : records){
            buf.clear();
            buf += ' '; // 未删除标记
            for(int fi=0;fi<numFld;fi++){
                int len = fields[fi].length;
                std::string val = (fi<(int)rec.values.size()) ? rec.values[fi] : "";
                char tp = fields[fi].type;
                if(tp=='N'||tp=='F'){
                    // 右对齐数值
                    if((int)val.size()>len) val=val.substr(0,len);
                    while((int)val.size()<len) val=" "+val;
                } else {
                    // 左对齐字符串
                    if((int)val.size()>len) val=val.substr(0,len);
                    while((int)val.size()<len) val+=" ";
                }
                buf += val;
            }
            f.write(buf.c_str(), buf.size());
        }
        f.put(0x1A); // EOF

        return true;
    }

    // ---- PRJ 文件（WGS84投影说明）----
    static bool writePrj(const std::string& path){
        std::ofstream f(path);
        if(!f){ std::cerr<<"[ERROR] Cannot open prj: "<<path<<"\n"; return false; }
        // 默认使用WGS84地理坐标系（投影坐标系时此处需定制）
        f << "GEOGCS[\"GCS_WGS_1984\","
          << "DATUM[\"D_WGS_1984\","
          << "SPHEROID[\"WGS_1984\",6378137.0,298.257223563]],"
          << "PRIMEM[\"Greenwich\",0.0],"
          << "UNIT[\"Degree\",0.017453292519943295]]";
        return true;
    }
};
